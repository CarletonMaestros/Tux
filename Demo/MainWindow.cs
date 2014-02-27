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

        bool viewport_changed = true;
        int viewport_width, viewport_height;

        Microsoft.Kinect.KinectSensor kinect;
        bool depth_image_update, skeleton_update;
        Microsoft.Kinect.DepthImageFrame depth_image, depth_image_new;
        Microsoft.Kinect.SkeletonFrame skeleton_frame, skeleton_frame_new;
        Microsoft.Kinect.Skeleton[] skeletons;
        Microsoft.Kinect.Skeleton skeleton;
        double last_skeleton_time;

        double time;
        Vector3 cam_pos, cam_tpos, cam_dpos;
        Vector3 cam_sub, cam_tsub, cam_dsub;
        double scam_theta, scam_y, scam_dtheta, scam_dy, scam_atheta;

        Microsoft.Kinect.SkeletonPoint last_hip;
        float[] last_ys = new float[32];
        bool[] last_beats = new bool[32];
        bool decreasing;

        enum CAM_FOCUS
        {
            SCENE,
            TEMPO
        }
        CAM_FOCUS cam_focus = CAM_FOCUS.SCENE;
        bool render_skeleton = true;
        bool render_tempo = true;

        public MainWindow()
        {
            WindowState = WindowState.Fullscreen;
            CursorVisible = false;

            Keyboard.KeyDown += delegate(object sender, KeyboardKeyEventArgs e)
            {
                if (e.Key == Key.Escape)
                    Exit();
                if (e.Key == Key.S)
                    render_skeleton = !render_skeleton;
                if (e.Key == Key.T)
                    render_tempo = !render_tempo;
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
                    depth_image_new = e.OpenDepthImageFrame();
                    depth_image_update = true;
                }
            };
            kinect.SkeletonFrameReady += delegate(object sender, Microsoft.Kinect.SkeletonFrameReadyEventArgs e)
            {
                lock (update_lock)
                {
                    skeleton_frame_new = e.OpenSkeletonFrame();
                    skeleton_update = true;
                }
            };
            kinect.Start();
        }

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
                float focus = 2000f;
                float distance = 8000f;
                float height = 2000f;
                scam_atheta += (0.5 - rand.NextDouble()) / 10 - scam_atheta / 100 - scam_dtheta / 1000;
                scam_dtheta += scam_atheta / 10;
                scam_dy += 1000 * rand.NextDouble() - scam_y / 2;
                scam_theta += scam_dtheta / 500;
                scam_y += scam_dy / 10000;
                cam_tpos = new Vector3((float)(distance * Math.Cos(scam_theta)), (float)scam_y + height, (float)(focus + distance * Math.Sin(scam_theta)));
                cam_tsub = new Vector3(0, 0, focus);
            }

            cam_pos = cam_tpos;
            cam_sub = cam_tsub;

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
                GL.Vertex3(new Vector3(1000 * last_hip.X + 500 + (last_ys.Length - i) * 50, last_hip.Y + last_ys[i] * 1000, 1000 * last_hip.Z));
            }
            GL.End();
            GL.LineWidth(4);
            GL.Begin(BeginMode.Lines);
            for (int i = 0; i < last_beats.Length; ++i)
            {
                if (last_beats[i])
                {
                    GL.Vertex3(new Vector3(1000 * last_hip.X + 500 + (last_beats.Length - i) * 50, last_hip.Y - 500 + last_ys[i] * 1000, 1000 * last_hip.Z));
                    GL.Vertex3(new Vector3(1000 * last_hip.X + 500 + (last_beats.Length - i) * 50, last_hip.Y + 500 + last_ys[i] * 1000, 1000 * last_hip.Z));
                }
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