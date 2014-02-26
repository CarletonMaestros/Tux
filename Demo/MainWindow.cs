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
        Thread rendering_thread;
        object update_lock = new object();
        bool exit = false;

        bool viewport_changed = true;
        int viewport_width, viewport_height;

        Microsoft.Kinect.KinectSensor kinect;
        Microsoft.Kinect.DepthImageFrame depth_image, depth_image_new;
        Microsoft.Kinect.SkeletonFrame skeleton_frame, skeleton_frame_new;
        Microsoft.Kinect.Skeleton[] skeletons;

        double time;
        Vector3 camera_pos;
        Vector3 camera_vel;
        double cam_theta, cam_y, cam_dtheta, cam_dy, cam_atheta;

        Random rand = new Random();

		public MainWindow()
		{
            WindowState = WindowState.Fullscreen;
            CursorVisible = false;

            Keyboard.KeyDown += delegate(object sender, KeyboardKeyEventArgs e)
            {
                if (e.Key == Key.Escape)
                    this.Exit();
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
            if (kinect == null) Exit();
            kinect.DepthStream.Enable();
            kinect.SkeletonStream.Enable();
            skeletons = new Microsoft.Kinect.Skeleton[kinect.SkeletonStream.FrameSkeletonArrayLength];
            kinect.DepthFrameReady += delegate(object sender, Microsoft.Kinect.DepthImageFrameReadyEventArgs e)
            {
                lock (update_lock) depth_image_new = e.OpenDepthImageFrame();
            };
            kinect.SkeletonFrameReady += delegate(object sender, Microsoft.Kinect.SkeletonFrameReadyEventArgs e)
            {
                lock (update_lock) skeleton_frame_new = e.OpenSkeletonFrame();
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
                Update(update_watch.Elapsed.TotalSeconds);
                update_watch.Reset();
                update_watch.Start();

                Render(render_watch.Elapsed.TotalSeconds);
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
                //if (depth_image != null) depth_image.Dispose();
                depth_image = depth_image_new;
                skeleton_frame = skeleton_frame_new;
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

            RenderPointCloud(dt);
            RenderSkeleton(dt);
        }

        void RenderPointCloud(double dt)
        {
            GL.PointSize(1);

            lock (update_lock)
            {
                Matrix4 projection = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 4, (float)(viewport_width / viewport_height), 1f, 100000f);
                GL.MatrixMode(MatrixMode.Projection);
                GL.LoadMatrix(ref projection);
            }

            float focus = 2000f;
            float distance = 8000f;
            float height = 2000f;
            cam_atheta += (0.5 - rand.NextDouble())/10 - cam_atheta/100 - cam_dtheta/1000;
            cam_dtheta += cam_atheta/10;
            cam_dy += 1000 * rand.NextDouble() - cam_y/2;
            cam_theta += cam_dtheta / 500;
            cam_y += cam_dy / 10000;
            Vector3 camera_pos = new Vector3((float)(distance * Math.Cos(cam_theta)), (float)cam_y+height, (float)(focus + distance * Math.Sin(cam_theta)));
            Matrix4 modelview = Matrix4.LookAt(camera_pos, new Vector3(0,0,focus), Vector3.UnitY);
            GL.MatrixMode(MatrixMode.Modelview);
            GL.LoadMatrix(ref modelview);

            GL.Begin(BeginMode.Points);
            GL.Color3(1f, 1f, 1f);
            if (depth_image != null)
            {
                var pixels = depth_image.GetRawPixelData();
                for (int i = 0; i < depth_image.PixelDataLength; i += 5)
                {
                    int x = i % depth_image.Width - depth_image.Width / 2;
                    int y = -(i / depth_image.Width - depth_image.Height / 2);
                    int z = pixels[i].Depth;
                    GL.Vertex3(x*z/500, y*z/500, z);
                }
            }

            GL.End();
        }

        void RenderSkeleton(double dt)
        {
            GL.LineWidth(2);

            if (skeleton_frame != null)
            {
                skeleton_frame.CopySkeletonDataTo(skeletons);
                for (int i = 0; i < skeleton_frame.SkeletonArrayLength; ++i)
                {
                    var skel = skeletons[i];
                    if (skel.TrackingState != Microsoft.Kinect.SkeletonTrackingState.Tracked) continue;

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
                }
            }
        }

        public void DrawBone(Microsoft.Kinect.Skeleton skel, Microsoft.Kinect.JointType a, Microsoft.Kinect.JointType b)
        {
            var lh = skel.Joints[a].Position;
            var rh = skel.Joints[b].Position;
            GL.Begin(BeginMode.Lines);
            GL.Color3(1f, 1f, 1f);
            GL.Vertex3(new Vector3(1000 * lh.X, 1000 * lh.Y, 1000 * lh.Z));
            GL.Vertex3(new Vector3(1000 * rh.X, 1000 * rh.Y, 1000 * rh.Z));
            GL.End();
        }

        #endregion

        #region public static void Main()

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
