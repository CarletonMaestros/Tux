using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Threading;

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;

namespace Orchestra
{
    /// <summary>
    /// Note that all OpenGL function calls should take place at the rendering thread -
    /// OpenGL will not be available on the main thread at all!
    /// </summary>
    public class MainWindow : GameWindow
    {
        Random rand = new Random();
        Thread rendering_thread;
        object update_lock = new object();
        bool exit = false;
        bool shift_down;

        bool viewport_changed = true;
        int viewport_width, viewport_height;

        Microsoft.Kinect.KinectSensor kinect;
        bool depth_image_update = false, skeleton_update = false;
        Microsoft.Kinect.DepthImageFrame depth_image, depth_image_new;
        Microsoft.Kinect.SkeletonFrame skeleton_frame, skeleton_frame_new;
        Microsoft.Kinect.Skeleton[] skeletons;
        Microsoft.Kinect.Skeleton skeleton;
        double last_skeleton_time;

        double time;
        float cam_speed;
        Vector3 cam_pos, cam_tpos, cam_dpos;
        Vector3 cam_sub, cam_tsub, cam_dsub;
        double scam_theta, scam_y, scam_dtheta, scam_dy, scam_atheta;

        Microsoft.Kinect.SkeletonPoint last_hip, last_head;
        double[] last_ys = new double[32];
        bool[] last_beats = new bool[32];
        bool decreasing;

        enum CAM_FOCUS
        {
            SCENE,
            SKELETON,
            TEMPO,
            FFT,
        }
        CAM_FOCUS cam_focus = CAM_FOCUS.SCENE;
        bool render_skeleton, render_tempo, render_fft;

        #region MainWindow()

        public MainWindow()
        {
            WindowState = WindowState.Fullscreen;
            CursorVisible = false;
            InitKinect();

            Keyboard.KeyDown += delegate(object sender, KeyboardKeyEventArgs e)
            {
                if (e.Key == Key.LShift || e.Key == Key.RShift)
                    shift_down = true;
                if (e.Key == Key.Escape)
                    Exit();
                if (e.Key == Key.S)
                    if (!shift_down) { render_skeleton = true; cam_focus = CAM_FOCUS.SKELETON; }
                    else { render_skeleton = false; if (cam_focus == CAM_FOCUS.SKELETON) cam_focus = CAM_FOCUS.SCENE; }
                if (e.Key == Key.T)
                    if (!shift_down) { render_tempo = true; render_fft = false; cam_focus = CAM_FOCUS.TEMPO; }
                    else { render_tempo = false; if (cam_focus == CAM_FOCUS.TEMPO) cam_focus = CAM_FOCUS.SCENE; }
                if (e.Key == Key.F)
                    if (!shift_down) { render_fft = true; render_tempo = false; cam_focus = CAM_FOCUS.FFT; }
                    else { render_tempo = false; if (cam_focus == CAM_FOCUS.FFT) cam_focus = CAM_FOCUS.SCENE; }
                if (e.Key == Key.Space)
                    cam_focus = CAM_FOCUS.SCENE;
            };

            Keyboard.KeyUp += delegate(object sender, KeyboardKeyEventArgs e)
            {
                if (e.Key == Key.LShift || e.Key == Key.RShift)
                    shift_down = false;
            };

            Resize += delegate(object sender, EventArgs e)
            {
                // Note that we cannot call any OpenGL methods directly. What we can do is set
                // a flag and respond to it from the rendering thread.
                lock (update_lock)
                {
                    viewport_changed = true;
                    viewport_width = Width;
                    viewport_height = Height;
                }
            };
        }

        #endregion

        #region InitKinect

        public void InitKinect()
        {
            foreach (var sensor in Microsoft.Kinect.KinectSensor.KinectSensors)
            {
                if (sensor.Status == Microsoft.Kinect.KinectStatus.Connected)
                {
                    kinect = sensor;
                    break;
                }
            }
            if (kinect == null) { Exit(); return; }
            kinect.DepthStream.Enable();
            kinect.SkeletonStream.Enable();
            skeletons = new Microsoft.Kinect.Skeleton[kinect.SkeletonStream.FrameSkeletonArrayLength];
            kinect.DepthFrameReady += delegate(object sender, Microsoft.Kinect.DepthImageFrameReadyEventArgs e)
            {
                lock (update_lock)
                {
                    if (depth_image_update && depth_image_new != null) depth_image_new.Dispose();
                    depth_image_new = e.OpenDepthImageFrame();
                    if (depth_image_new != null) depth_image_update = true;
                }
            };
            kinect.SkeletonFrameReady += delegate(object sender, Microsoft.Kinect.SkeletonFrameReadyEventArgs e)
            {
                lock (update_lock)
                {
                    if (skeleton_update && skeleton_frame_new != null) skeleton_frame_new.Dispose();
                    skeleton_frame_new = e.OpenSkeletonFrame();
                    if (skeleton_frame_new != null) skeleton_update = true;
                }
            };
            kinect.Start();
        }

        #endregion

        #region OnLoad

        /// <summary>
        /// Setup OpenGL and load resources here.
        /// </summary>
        /// <param name="e">Not used.</param>
        protected override void OnLoad(EventArgs e)
        {
            Context.MakeCurrent(null); // Release the OpenGL context so it can be used on the new thread.

            rendering_thread = new Thread(RenderLoop);
            rendering_thread.IsBackground = true;
            rendering_thread.Start();
        }

        #endregion

        #region OnUnload

        /// <summary>
        /// Release resources here.
        /// </summary>
        /// <param name="e">Not used.</param>
        protected override void OnUnload(EventArgs e)
        {
            exit = true; // Set a flag that the rendering thread should stop running.
            rendering_thread.Join();

            base.OnUnload(e);
        }

        #endregion

        #region RenderLoop

        void RenderLoop()
        {
            MakeCurrent(); // The context now belongs to this thread. No other thread may use it!

            VSync = VSyncMode.On;

            // Since we don't use OpenTK's timing mechanism, we need to keep time ourselves;
            Stopwatch render_watch = new Stopwatch();
            Stopwatch update_watch = new Stopwatch();
            update_watch.Start();
            render_watch.Start();

            while (!exit)
            {
                Update(Math.Min(.1, update_watch.Elapsed.TotalSeconds));
                update_watch.Reset();
                update_watch.Start();

                Render(Math.Min(.1, render_watch.Elapsed.TotalSeconds));
                render_watch.Reset(); //  Stopwatch may be inaccurate over larger intervals.
                render_watch.Start(); // Plus, timekeeping is easier if we always start counting from 0.

                SwapBuffers();
            }

            Context.MakeCurrent(null);
        }

        #endregion

        #region Update

        void Update(double dt)
        {
            time += dt;

            lock (update_lock)
            {
                if (depth_image_update)
                {
                    depth_image_update = false;
                    if (depth_image != null) depth_image.Dispose();
                    depth_image = depth_image_new;
                }
            }

            lock (update_lock)
            {
                if (skeleton_update)
                {
                    skeleton_update = false;
                    if (skeleton_frame != null) skeleton_frame.Dispose();
                    skeleton_frame = skeleton_frame_new;

                    skeleton_frame.CopySkeletonDataTo(skeletons);
                    last_skeleton_time = time;
                    skeleton = null;
                    for (int i = 0; i < skeleton_frame.SkeletonArrayLength; ++i)
                    {
                        if (skeletons[i].TrackingState == Microsoft.Kinect.SkeletonTrackingState.Tracked)
                        {
                            skeleton = skeletons[i];
                            break;
                        }
                    }

                    if (skeleton != null)
                    {
                        for (int i = 0; i < last_ys.Length - 1; ++i)
                            last_ys[i] = last_ys[i + 1];
                        if (skeleton != null && skeleton.Joints[Microsoft.Kinect.JointType.HandRight].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked && skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked)
                            last_ys[last_ys.Length - 1] = skeleton.Joints[Microsoft.Kinect.JointType.HandRight].Position.Y - skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].Position.Y;
                        else
                            last_ys[last_ys.Length - 1] = float.NaN;
                        if (skeleton != null && skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked)
                            last_hip = skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].Position;
                        if (skeleton != null && skeleton.Joints[Microsoft.Kinect.JointType.Head].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked)
                            last_head = skeleton.Joints[Microsoft.Kinect.JointType.Head].Position;
                        for (int i = 0; i < last_beats.Length - 1; ++i)
                            last_beats[i] = last_beats[i + 1];
                        if (decreasing && last_ys[last_ys.Length - 3] < last_ys[last_ys.Length - 2] && last_ys[last_ys.Length - 2] < last_ys[last_ys.Length - 1])
                        {
                            last_beats[last_beats.Length - 1] = true;
                            decreasing = false;
                        }
                        else
                            last_beats[last_beats.Length - 1] = false;
                        if (!decreasing && last_ys[last_ys.Length - 3] > last_ys[last_ys.Length - 2] && last_ys[last_ys.Length - 2] > last_ys[last_ys.Length - 1])
                            decreasing = true;
                    }
                }
            }
        }

        #endregion

        #region Render

        /// <summary>
        /// This is our main rendering function, which executes on the rendering thread.
        /// </summary>
        public void Render(double dt)
        {
            GL.ClearColor(Color.Black);
            GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);

            lock (update_lock)
            {
                if (viewport_changed)
                {
                    GL.Viewport(0, 0, viewport_width, viewport_height);
                    viewport_changed = false;
                }
            }

            PositionCamera(dt);
            RenderPointCloud(dt);
            RenderSkeleton(dt);
            RenderTempo(dt);
            RenderFFT(dt);
        }

        #endregion

        #region Camera

        void PositionCamera(double dt)
        {
            // Update viewport
            lock (update_lock)
            {
                Matrix4 projection = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 4, (float)(viewport_width / viewport_height), 1f, 100000f);
                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadMatrix(ref projection);
            }

            if (cam_focus == CAM_FOCUS.SCENE)
            {
                float focus = 2000;
                float distance = 8000;
                float height = 2000;
                cam_speed = 1;
                scam_theta += (float)(dt) / (0.5-rand.NextDouble()) / 10;
                cam_tpos = new Vector3((float)(distance * Math.Cos(scam_theta)), (float)height, (float)(focus + distance * Math.Sin(scam_theta)));
                cam_tsub = new Vector3(0, 0, focus);
            }
            else if (cam_focus == CAM_FOCUS.SKELETON)
            {
                float height = 1000;
                float distance = 4000;
                cam_speed = 50;
                cam_tsub = new Vector3(last_hip.X * 1000, last_hip.Y * 1000 + 500, last_hip.Z * 1000);
                cam_tpos = cam_tsub + new Vector3(0, height, -distance);
            }
            else if (cam_focus == CAM_FOCUS.TEMPO)
            {
                float height = 1000;
                float distance = 4000;
                cam_speed = 50;
                cam_tsub = new Vector3(last_hip.X * 1000 + 1250, last_hip.Y * 1000 + 500, last_hip.Z * 1000);
                cam_tpos = cam_tsub + new Vector3(0, height, -distance);
            }
            else if (cam_focus == CAM_FOCUS.FFT)
            {
                float height = 1000;
                float distance = 4000;
                cam_speed = 50;
                cam_tsub = new Vector3(last_head.X * 1000, last_head.Y * 1000 + 1000, last_head.Z * 1000);
                cam_tpos = cam_tsub + new Vector3(0, height, -distance);
            }

            Vector3 dp = cam_tpos - cam_pos;
            cam_dpos += (float)(dt) * cam_speed * ((dp + dp / dp.Length * 20 * (float)Math.Sqrt(dp.Length)) / 10 - cam_dpos / 10);
            cam_pos += (float)(dt) * cam_dpos;
            Vector3 ds = cam_tsub - cam_sub;
            cam_dsub += (float)(dt) * cam_speed * ((ds + ds / ds.Length * 20 * (float)Math.Sqrt(ds.Length)) / 10 - cam_dsub / 10);
            cam_sub += (float)(dt) * cam_dsub;
            //cam_pos = cam_tpos;
            //cam_sub = cam_tsub;

            Matrix4 modelview = Matrix4.LookAt(cam_pos, cam_sub, Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref modelview);
        }

        #endregion

        #region PointCloud

        void RenderPointCloud(double dt)
        {
            if (depth_image == null) return;

            GL.PointSize(1);
            GL.Color3(1f, 1f, 1f);
            GL.Begin(BeginMode.Points);
            var pixels = depth_image.GetRawPixelData();
            int w = depth_image.Width;
            int h = depth_image.Height;
            int hw = w / 2;
            int hh = h / 2;
            for (int i = 0; i < depth_image.PixelDataLength; i += 5)
            {
                int x = i % w - hw;
                int y = hh - i / w;
                int z = pixels[i].Depth;
                GL.Vertex3(x*z/500, y*z/500, z);
            }
            GL.End();
        }

        #endregion

        #region Skeleton

        void RenderSkeleton(double dt)
        {
            if (skeleton_frame == null || !render_skeleton) return;

            GL.LineWidth(2);
            for (int i = 0; i < skeleton_frame.SkeletonArrayLength; ++i)
            {
                var skel = skeletons[i];
                if (skel.TrackingState != Microsoft.Kinect.SkeletonTrackingState.Tracked) continue;

                GL.Color3(1f, 1f, 1f);
                GL.Begin(BeginMode.Lines);

                // Render Torso
                DrawBone(skel, Microsoft.Kinect.JointType.Head, Microsoft.Kinect.JointType.ShoulderCenter);
                DrawBone(skel, Microsoft.Kinect.JointType.ShoulderCenter, Microsoft.Kinect.JointType.ShoulderLeft);
                DrawBone(skel, Microsoft.Kinect.JointType.ShoulderCenter, Microsoft.Kinect.JointType.ShoulderRight);
                DrawBone(skel, Microsoft.Kinect.JointType.ShoulderCenter, Microsoft.Kinect.JointType.Spine);
                DrawBone(skel, Microsoft.Kinect.JointType.Spine, Microsoft.Kinect.JointType.HipCenter);
                DrawBone(skel, Microsoft.Kinect.JointType.HipCenter, Microsoft.Kinect.JointType.HipLeft);
                DrawBone(skel, Microsoft.Kinect.JointType.HipCenter, Microsoft.Kinect.JointType.HipRight);

                // Left Arm
                DrawBone(skel, Microsoft.Kinect.JointType.ShoulderLeft, Microsoft.Kinect.JointType.ElbowLeft);
                DrawBone(skel, Microsoft.Kinect.JointType.ElbowLeft, Microsoft.Kinect.JointType.WristLeft);
                DrawBone(skel, Microsoft.Kinect.JointType.WristLeft, Microsoft.Kinect.JointType.HandLeft);

                // Right Arm
                DrawBone(skel, Microsoft.Kinect.JointType.ShoulderRight, Microsoft.Kinect.JointType.ElbowRight);
                DrawBone(skel, Microsoft.Kinect.JointType.ElbowRight, Microsoft.Kinect.JointType.WristRight);
                DrawBone(skel, Microsoft.Kinect.JointType.WristRight, Microsoft.Kinect.JointType.HandRight);

                // Left Leg
                DrawBone(skel, Microsoft.Kinect.JointType.HipLeft, Microsoft.Kinect.JointType.KneeLeft);
                DrawBone(skel, Microsoft.Kinect.JointType.KneeLeft, Microsoft.Kinect.JointType.AnkleLeft);
                DrawBone(skel, Microsoft.Kinect.JointType.AnkleLeft, Microsoft.Kinect.JointType.FootLeft);

                // Right Leg
                DrawBone(skel, Microsoft.Kinect.JointType.HipRight, Microsoft.Kinect.JointType.KneeRight);
                DrawBone(skel, Microsoft.Kinect.JointType.KneeRight, Microsoft.Kinect.JointType.AnkleRight);
                DrawBone(skel, Microsoft.Kinect.JointType.AnkleRight, Microsoft.Kinect.JointType.FootRight);

                GL.End();
            }
        }

        public void DrawBone(Microsoft.Kinect.Skeleton skel, Microsoft.Kinect.JointType a, Microsoft.Kinect.JointType b)
        {
            var lh = skel.Joints[a].Position;
            var rh = skel.Joints[b].Position;
            GL.Vertex3(new Vector3(1000 * lh.X, 1000 * lh.Y, 1000 * lh.Z));
            GL.Vertex3(new Vector3(1000 * rh.X, 1000 * rh.Y, 1000 * rh.Z));
        }

        #endregion

        #region Tempo

        public void RenderTempo(double dt)
        {
            if (skeleton == null || !render_tempo) return;

            GL.PointSize(4);
            GL.Color3(1f, 1f, 1f);
            GL.Begin(BeginMode.Points);
            for (int i = 0; i < last_ys.Length; ++i)
            {
                GL.Vertex3(new Vector3(1000 * last_hip.X + 500 + 50 * (last_ys.Length - i), 1000 * (last_hip.Y + (float)last_ys[i]), 1000 * last_hip.Z));
            }
            GL.End();
            GL.LineWidth(4);
            GL.Begin(BeginMode.Lines);
            for (int i = 0; i < last_beats.Length; ++i)
            {
                if (last_beats[i])
                {
                    GL.Vertex3(new Vector3(1000 * last_hip.X + 500 + (last_beats.Length - i) * 50, 1000 * (last_hip.Y + (float)last_ys[i]) + 500, 1000 * last_hip.Z));
                    GL.Vertex3(new Vector3(1000 * last_hip.X + 500 + (last_beats.Length - i) * 50, 1000 * (last_hip.Y + (float)last_ys[i]) - 500, 1000 * last_hip.Z));
                }
            }
            GL.End();
        }

        #endregion

        #region FFT

        public void RenderFFT(double dt)
        {
            if (skeleton == null || !render_fft) return;

            GL.PointSize(4);
            GL.Color3(1f, 1f, 1f);
            GL.Begin(BeginMode.Points);
            for (int i = 0; i < last_ys.Length; ++i)
            {
                GL.Vertex3(new Vector3(1000 * last_head.X - 750 + 50 * i, 1000 * (last_head.Y + (float)last_ys[i]) + 1000, 1000 * last_head.Z));
            }
            GL.End();
            GL.LineWidth(1);
            GL.Begin(BeginMode.Lines);
            for (int i = 0; i < last_beats.Length; ++i)
            {
                if (last_beats[i])
                {
                    GL.Vertex3(new Vector3(1000 * last_head.X - 750 + i * 50, 1000 * (last_head.Y + (float)last_ys[i]) + 1500, 1000 * last_head.Z));
                    GL.Vertex3(new Vector3(1000 * last_head.X - 750 + i * 50, 1000 * (last_head.Y + (float)last_ys[i]) + 500, 1000 * last_head.Z));
                }
            }
            GL.End();
            GL.LineWidth(4);
            GL.Begin(BeginMode.LineStrip);
            float a = (float)(MathNet.Numerics.Statistics.Statistics.StandardDeviation(last_ys) / Math.Sqrt(Math.PI));
            float b = (float)MathNet.Numerics.Statistics.Statistics.Mean(last_ys);
            int p = 0;
            for (int i = 1; i < last_ys.Length; ++i)
            {
                if (last_ys[i] < last_ys[p]) p = i;
            }
            for (int i = 0; i < last_beats.Length * 10; ++i)
            {
                GL.Vertex3(new Vector3(1000 * last_head.X - 750 + i * 5, 1000 * (last_head.Y + 2*a*(float)Math.Sin((i-10*p)/10f) + b) + 1000, 1000 * last_head.Z));
            }
            GL.End();
        }

        #endregion

        #region Main

        /// <summary>
        /// Entry point.
        /// </summary>
        [STAThread]
        public static void Main()
        {
            using (GameWindow window = new MainWindow())
            {
                window.Run();
            }
        }

        #endregion
    }
}