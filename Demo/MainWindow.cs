using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Numerics;
using System.Timers;

using OpenTK;
using OpenTK.Graphics;
using OpenTK.Graphics.OpenGL;
using OpenTK.Input;

namespace Orchestra
{
	// Threaded OpenGL window.
	// All rendering occurs on a background thread.
	public class MainWindow : GameWindow
	{
		System.Threading.Thread render_thread;
		Random rand = new Random();
		Stopwatch stopwatch;
		Timer timer;
		State s, rs;

		Microsoft.Kinect.KinectSensor kinect;

		// The current state of the window.
		// Main and render threads maintain independent State objects.
		// Each frame, the main state object is locked and copied to the render state.
		partial class State
		{
			public MainWindow window;
			public bool should_exit;
			public bool shift_key_down;
			public int viewport_width, viewport_height;

			// Time of last updates
			public double camera_time;
			public double depth_time;
			public double skeleton_time;

			// Camera state (cam_*=current, cam_t*=target, cam_d*=velocity)
			public CAM_FOCUS cam_focus = CAM_FOCUS.SCENE; // Camera mode
			public Vector3 cam_pos, cam_tpos, cam_dpos; // Camera position
			public Vector3 cam_sub, cam_tsub, cam_dsub; // Camera subject (focus point)
			public double scam_theta, scam_y, scam_dtheta, scam_dy, scam_atheta; // Scene camera

			// Kinect data
			public bool depth_image_copied, skeleton_frame_copied;
			public Microsoft.Kinect.DepthImageFrame depth_image;
			public Microsoft.Kinect.SkeletonFrame skeleton_frame;
			public Microsoft.Kinect.Skeleton[] skeletons;
			public Microsoft.Kinect.Skeleton skeleton;
			public Microsoft.Kinect.SkeletonPoint hip, head;
			public double[] last_ys = new double[32];
			public bool[] last_beats = new bool[32];
			public bool hand_falling;

			// Overlays
			public bool render_skeleton, render_fft;
			public int fft_slide = -1;

			public State(MainWindow window)
			{
				this.window = window;
			}

			public void CopyTo(State dest)
			{
				lock (this)
				{
					lock (dest)
					{
						dest.should_exit = should_exit;
						dest.shift_key_down = shift_key_down;
						dest.viewport_width = viewport_width;
						dest.viewport_height = viewport_height;

						dest.camera_time = camera_time;
						dest.depth_time = depth_time;
						dest.skeleton_time = skeleton_time;

						dest.cam_focus = cam_focus;
						dest.cam_pos = cam_pos;
						dest.cam_tpos = cam_tpos;
						dest.cam_dpos = cam_dpos;
						dest.cam_sub = cam_sub;
						dest.cam_tsub = cam_tsub;
						dest.cam_dsub = cam_dsub;

						if (depth_image != null && depth_image != dest.depth_image)
						{
							if (dest.depth_image != null)
								dest.depth_image.Dispose();
							dest.depth_image = depth_image;
							depth_image_copied = true;
						}

						if (skeleton_frame != null && skeleton_frame != dest.skeleton_frame)
						{
							if (dest.skeleton_frame != null)
								dest.skeleton_frame.Dispose();
							dest.skeleton_frame = skeleton_frame;
							skeleton_frame_copied = true;
						}

						if (skeletons != null && dest.skeletons != null) Array.Copy(skeletons, dest.skeletons, skeleton_frame.SkeletonArrayLength);
						dest.skeleton = skeleton;
						Array.Copy(last_ys, dest.last_ys, last_ys.Length);
						Array.Copy(last_beats, dest.last_beats, last_beats.Length);
						dest.hand_falling = hand_falling;

						dest.render_skeleton = render_skeleton;
						dest.render_fft = render_fft;
						dest.fft_slide = fft_slide;
					}
				}
			}
		}

		enum CAM_FOCUS
		{
			SCENE,
			SKELETON,
			FFT,
		}

		public MainWindow()
		{
			s = new State(this);
			s.cam_pos = new Vector3(8000, 2000, 2000);
			s.cam_sub = new Vector3(0, 0, 2000);
			rs = new State(this);

			stopwatch = Stopwatch.StartNew();
			timer = new Timer(1000/60);
			timer.Elapsed += s.Update();
			timer.AutoReset = true;
			timer.Enabled = true;

			Keyboard.KeyDown += s.KeyDown;
			Keyboard.KeyUp += s.KeyUp;
			Resize += s.Resize;

			WindowState = WindowState.Fullscreen;
			CursorVisible = false;
			InitKinect();
		}

		void InitKinect()
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
			s.skeletons = new Microsoft.Kinect.Skeleton[kinect.SkeletonStream.FrameSkeletonArrayLength];
			rs.skeletons = new Microsoft.Kinect.Skeleton[kinect.SkeletonStream.FrameSkeletonArrayLength];
			kinect.DepthFrameReady += s.DepthFrameReady;
			kinect.SkeletonFrameReady += s.SkeletonFrameReady;
			kinect.Start();
		}

		partial class State
		{
			public void KeyDown(object sender, KeyboardKeyEventArgs e)
			{
				if (e.Key == Key.LShift || e.Key == Key.RShift)
					shift_key_down = true;
				if (e.Key == Key.Escape)
					window.Exit();
				if (e.Key == Key.S)
				if (!shift_key_down) { render_skeleton = true; cam_focus = CAM_FOCUS.SKELETON; }
				else { render_skeleton = false; if (cam_focus == CAM_FOCUS.SKELETON) cam_focus = CAM_FOCUS.SCENE; }
				if (e.Key == Key.T)
				if (!shift_key_down) { render_fft = true; fft_slide = 0; cam_focus = CAM_FOCUS.FFT; }
				else { render_fft = false; fft_slide = -1; if (cam_focus == CAM_FOCUS.FFT) cam_focus = CAM_FOCUS.SCENE; }
				if (e.Key == Key.F)
				if (!shift_key_down) { render_fft = true; fft_slide++; cam_focus = CAM_FOCUS.FFT; }
				else { render_fft = false; fft_slide = -1; if (cam_focus == CAM_FOCUS.FFT) cam_focus = CAM_FOCUS.SCENE; }
				if (e.Key == Key.Space)
					cam_focus = CAM_FOCUS.SCENE;
			}

			public void KeyUp(object sender, KeyboardKeyEventArgs e)
			{
				if (e.Key == Key.LShift || e.Key == Key.RShift)
					shift_key_down = false;
			}

			public void Resize(object sender, EventArgs e)
			{
				viewport_width = window.Width;
				viewport_height = window.Height;
			}

			public void DepthFrameReady(object sender, Microsoft.Kinect.DepthImageFrameReadyEventArgs e)
			{
				depth_image = e.OpenDepthImageFrame();
				if (depth_image == null) return;
				depth_time = window.stopwatch.Elapsed.TotalSeconds;
			}

			public void SkeletonFrameReady(object sender, Microsoft.Kinect.SkeletonFrameReadyEventArgs e)
			{
				skeleton_frame = e.OpenSkeletonFrame();
				if (skeleton_frame == null) return;
				skeleton_time = window.stopwatch.Elapsed.TotalSeconds;

				skeleton_frame.CopySkeletonDataTo(skeletons);
				skeleton_time = window.stopwatch.Elapsed.TotalSeconds;
				skeleton = null;
				for (int i = 0; i < skeleton_frame.SkeletonArrayLength; ++i)
				{
					if (skeletons[i].TrackingState == Microsoft.Kinect.SkeletonTrackingState.Tracked)
					{
						skeleton = skeletons[i];
						break;
					}
				}
				if (skeleton == null) return;

				for (int i = 0; i < last_ys.Length - 1; ++i)
					last_ys[i] = last_ys[i + 1];
				if (skeleton != null && skeleton.Joints[Microsoft.Kinect.JointType.HandRight].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked && skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked)
					last_ys[last_ys.Length - 1] = skeleton.Joints[Microsoft.Kinect.JointType.HandRight].Position.Y - skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].Position.Y;
				else
					last_ys[last_ys.Length - 1] = float.NaN;
				if (skeleton != null && skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked)
					hip = skeleton.Joints[Microsoft.Kinect.JointType.HipCenter].Position;
				if (skeleton != null && skeleton.Joints[Microsoft.Kinect.JointType.Head].TrackingState != Microsoft.Kinect.JointTrackingState.NotTracked)
					head = skeleton.Joints[Microsoft.Kinect.JointType.Head].Position;
				for (int i = 0; i < last_beats.Length - 1; ++i)
					last_beats[i] = last_beats[i + 1];
				if (hand_falling && last_ys[last_ys.Length - 3] < last_ys[last_ys.Length - 2] && last_ys[last_ys.Length - 2] < last_ys[last_ys.Length - 1])
				{
					last_beats[last_beats.Length - 1] = true;
					hand_falling = false;
				}
				else
					last_beats[last_beats.Length - 1] = false;
				if (!hand_falling && last_ys[last_ys.Length - 3] > last_ys[last_ys.Length - 2] && last_ys[last_ys.Length - 2] > last_ys[last_ys.Length - 1])
					hand_falling = true;
			}

			public void Update(object sender, ElapsedEventArgs e)
			{
				UpdateCamera();
			}

			public void UpdateCamera()
			{
				float dt = (float)window.stopwatch.Elapsed.TotalSeconds - camera_time;
				camera_time = window.stopwatch.Elapsed.TotalSeconds;

				float cam_speed = 1;
				if (cam_focus == CAM_FOCUS.SCENE)
				{
					float focus = 2000;
					float distance = 8000;
					float height = 2000;
					scam_theta += dt / (0.5-window.rand.NextDouble()) / 10;
					cam_tpos = new Vector3((float)(distance * Math.Cos(scam_theta)), (float)height, (float)(focus + distance * Math.Sin(scam_theta)));
					cam_tsub = new Vector3(0, 0, focus);
				}
				else if (cam_focus == CAM_FOCUS.SKELETON)
				{
					float height = 1000;
					float distance = 4000;
					cam_speed = 50;
					cam_tsub = new Vector3(hip.X * 1000, hip.Y * 1000 + 500, hip.Z * 1000);
					cam_tpos = cam_tsub + new Vector3(0, height, -distance);
				}
				else if (cam_focus == CAM_FOCUS.FFT)
				{
					float height = 1000;
					float distance = 4000;
					cam_speed = 50;
					cam_tsub = new Vector3(head.X * 1000, head.Y * 1000 + 1000, head.Z * 1000);
					cam_tpos = cam_tsub + new Vector3(0, height, -distance);
				}

				Vector3 dp = cam_tpos - cam_pos;
				if (dp.Length > 1)
					cam_dpos += dt * cam_speed * ((dp + dp / dp.Length * 20 * (float)Math.Sqrt(dp.Length)) / 10 - cam_dpos / 10);
				cam_pos += dt * cam_dpos;
				Vector3 ds = cam_tsub - cam_sub;
				if (ds.Length > 1)
					cam_dsub += dt * cam_speed * ((ds + ds / ds.Length * 20 * (float)Math.Sqrt(ds.Length)) / 10 - cam_dsub / 10);
				cam_sub += dt * cam_dsub;
			}
		}

		protected override void OnLoad(EventArgs e)
		{
			Context.MakeCurrent(null); // Release the OpenGL context so it can be used on the new thread.

			render_thread = new System.Threading.Thread(RenderLoop);
			render_thread.IsBackground = true;
			render_thread.Start();
		}

		protected override void OnUnload(EventArgs e)
		{
			lock (s)
			{
				s.should_exit = true; // Set a flag that the rendering thread should stop running.
			}
			render_thread.Join();

			base.OnUnload(e);
		}

		void RenderLoop()
		{
			MakeCurrent(); // The context now belongs to this thread. No other thread may use it!
			VSync = VSyncMode.On;

			while (!rs.should_exit)
			{
				s.CopyTo(rs);
				rs.Render();
				SwapBuffers();
			}

			Context.MakeCurrent(null);
		}

		partial class State
		{
			public void Render()
			{
				GL.ClearColor(Color.Black);
				GL.Clear(ClearBufferMask.ColorBufferBit | ClearBufferMask.DepthBufferBit);
				GL.Viewport(0, 0, viewport_width, viewport_height);

				SetupCamera();
				RenderPointCloud();
				RenderSkeleton();
				RenderFFT();
			}

			void SetupCamera()
			{
				Matrix4 projection = Matrix4.CreatePerspectiveFieldOfView((float)Math.PI / 4, (float)(viewport_width / viewport_height), 1f, 100000f);
				GL.MatrixMode(MatrixMode.Projection);
				GL.LoadMatrix(ref projection);

				float dt = (float)window.stopwatch.Elapsed.TotalSeconds - camera_time;
				Vector3 pos = cam_pos + dt * cam_dpos;
				Vector3 sub = cam_sub + dt * cam_dsub;

				Matrix4 modelview = Matrix4.LookAt(pos, sub, Vector3.UnitY);
				GL.MatrixMode(MatrixMode.Modelview);
				GL.LoadMatrix(ref modelview);
			}

			void RenderPointCloud()
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
				for (int i = 0; i < depth_image.PixelDataLength; i += 7)
				{
					int x = i % w - hw;
					int y = hh - i / w;
					int z = pixels[i].Depth;
					GL.Vertex3(x*z/500, y*z/500, z);
				}
				GL.End();
			}

			void RenderSkeleton()
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

			public void RenderFFT()
			{
				if (skeleton == null || !render_fft) return;

				float skel_time = (float)Math.Min(1, 30 * (window.stopwatch.Elapsed.TotalSeconds - skeleton_time));

				GL.PointSize(4);
				GL.Color3(1f, 1f, 1f);
				GL.Begin(BeginMode.Points);
				for (int i = 0; i < last_ys.Length; ++i)
				{
					if (last_ys[i] != last_ys[i]) continue;
					GL.Vertex3(new Vector3(1000 * head.X - 750 + 50 * (i - skel_time), 1000 * (head.Y + (float)last_ys[i]) + 1000, 1000 * head.Z));
				}
				GL.End();
				GL.LineWidth(4);
				GL.Begin(BeginMode.Lines);
				for (int i = 0; i < last_beats.Length; ++i)
				{
					if (last_beats[i])
					{
						GL.Vertex3(new Vector3(1000 * head.X - 750 + 50 * (i - skel_time), 1000 * (head.Y + (float)last_ys[i]) + 1500, 1000 * head.Z));
						GL.Vertex3(new Vector3(1000 * head.X - 750 + 50 * (i - skel_time), 1000 * (head.Y + (float)last_ys[i]) + 500, 1000 * head.Z));
					}
				}
				GL.End();
				float a = (float)(MathNet.Numerics.Statistics.Statistics.StandardDeviation(last_ys) / Math.Sqrt(Math.PI));
				float b = (float)MathNet.Numerics.Statistics.Statistics.Mean(last_ys);
				Complex[] ys = new Complex[last_ys.Length];
				for (int i = 0; i < last_ys.Length; ++i)
				{
					ys[i] = last_ys[i];
				}
				MathNet.Numerics.IntegralTransforms.Transform.FourierForward(ys);
				int f0 = 1;
				for (int i = 2; i < ys.Length / 2; ++i)
				{
					if (ys[i].Magnitude > ys[f0].Magnitude) f0 = i;
				}
				int p = 0;
				for (int i = 1; i < last_ys.Length; ++i)
				{
					if (last_ys[i] < last_ys[p]) p = i;
				}
				double minerr = double.MaxValue;
				int mini = 0;
				for (int i = 0; i <= 100; ++i)
				{
					double err = 0;
					for (int j = 0; j < last_ys.Length; ++j)
					{
						if (last_ys[j] != last_ys[j]) continue;
						double er = -2*a*Math.Cos((j-p)*1.0/last_ys.Length*(f0-0.5+i/100f)*2*Math.PI) + b - last_ys[j];
						err += er * er;
					};
					if (err < minerr) { minerr = err; mini = i; }
				}
				float f = f0 - 0.5f + mini/100f;
				if (fft_slide < 1) return;
				if (fft_slide < 2) a = 0.25f;
				if (fft_slide < 3) b = 0;
				if (fft_slide < 4) p = 0;
				if (fft_slide < 5) f = f0;
				GL.LineWidth(2);
				GL.Color3(1f, 0f, 0f);
				GL.Begin(BeginMode.LineStrip);
				for (int i = 0; i < last_ys.Length * 10; ++i)
				{
					GL.Vertex3(new Vector3(1000 * head.X - 750 + i * 5 - 50 * skel_time, 1000 * (head.Y - 2*a*(float)Math.Cos((i/10f-p)/last_ys.Length*f*2*(float)Math.PI) + b) + 1000, 1000 * head.Z));
				}
				GL.End();
				GL.LineWidth(4);
				GL.Begin(BeginMode.Lines);
				for (int i = 0; i < f + 1; ++i)
				{
					float height = 1000;
					GL.Vertex3(new Vector3(1000 * head.X - 750 + 50 * last_ys.Length * (1f * p / last_ys.Length % (1/f) + i / f) - 50 * skel_time, 1000 * (head.Y + b - 2 * a) + 1000 - height / 2, 1000 * head.Z));
					GL.Vertex3(new Vector3(1000 * head.X - 750 + 50 * last_ys.Length * (1f * p / last_ys.Length % (1/f) + i / f) - 50 * skel_time, 1000 * (head.Y + b - 2 * a) + 1000 + height / 2, 1000 * head.Z));
				}
				GL.End();
			}
		}

		[STAThread]
		public static void Main()
		{
			using (GameWindow window = new MainWindow())
			{
				window.Run();
			}
		}
	}
}