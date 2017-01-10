using System;
using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using System.Linq;
using System.Windows.Threading;
using System.IO;

namespace KinectV2
{
    /// <summary>
    /// MainWindow.xaml の相互作用ロジック
    /// </summary>
    public partial class MainWindow : Window
    {
        KinectSensor kinect;
        BodyFrameReader bodyFrameReader;
        Body[] bodies = null;
        int bodyCount;

        FaceFrameSource[] faceFrameSources = null;
        FaceFrameReader[] faceFrameReaders = null;
        FaceFrameResult[] faceFrameResults = null;

        ColorFrameReader colorFrameReader;
        FrameDescription colorFrameDesc;

        ColorImageFormat colorFormat = ColorImageFormat.Bgra;

        // WPF
        WriteableBitmap colorBitmap;
        byte[] colorBuffer;
        int colorStride;
        Int32Rect colorRect;

        private DispatcherTimer _timer = null;
        private DateTime _startTime;
        TimeSpan ElapsedTime;
        DateTime Nowpast;

        //関節名と座標を格納する。JC=JointCoodinateの略。
        Dictionary<string, double[]> JC = new Dictionary<string, double[]>();
        //関節毎に閾値を設ける。JT=JointThresholdの略
        Dictionary<string, double> JT = new Dictionary<string, double>();

        //距離関数で使う引数を定義しておく。
        double squaredistance, distance;

        //角度変化関数returnPYRで使用する引数に初期値を代入。
        double PITCH, pitch, opitch = 0.0;
        double ROLL, roll, oroll = 0.0;
        double YAW, yaw, oyaw = 0.0;

        StreamWriter sw = new StreamWriter(@"C:\Users\eriguchi\Desktop\record.csv");
        string colnames = "Time,SpineBase,SpineMid,Neck,Head,ShoulderLeft,ElbowLeft,WristLeft,HandLeft,ShoulderRight,ElbowRight,WristRight,HandRight,HipLeft,KneeLeft,AnkleLeft,FootLeft,HipRight,KneeRight,AnkleRight,FootRight,SpineShoulder,HandTipLeft,ThumbLeft,HandTipRight,ThumbRight,Pitch,Yaw,Roll";


        public MainWindow()
        {
            InitializeComponent();
            sw.WriteLine(colnames);
        }

        public void _timer_Tick(object sender, EventArgs e)
        {
            string linevector = "";

            string hour = (DateTime.Now - this._startTime).Hours.ToString();
            string minute = (DateTime.Now - this._startTime).Minutes.ToString();
            double second = (DateTime.Now - this._startTime).Seconds + (DateTime.Now - this._startTime).Milliseconds / 1000.0;
            string sec = second.ToString();
            string present = hour + ":" + minute + ":" + sec;
            linevector = present;
           

            string zeroline = ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";


            try
            {
                ElapsedTime = DateTime.Now - Nowpast;
                double ET = ElapsedTime.TotalSeconds;

                if (bodies != null)
                {
                    foreach (var body in bodies.Where(b => b.IsTracked))
                    {
                        foreach (Joint joint in body.Joints.Values)
                        {
                            string ThisJoint = joint.JointType.ToString();
                            double Velocity = Distance(body, joint) / (ET * 10);
                            int V = (int)Math.Floor(Velocity);
                            V = Math.Abs(V);
                            linevector = linevector + "," + V.ToString();

                            Nowpast = DateTime.Now;
                        }
                    }
                    
                }
                else
                {
                    linevector = linevector + zeroline;
                    TBone.Text = "Untracked" + "\t" + present;

                }

                //同一人物でもIDが変更されていることがあるため。
                for (int i = 0; i < bodyCount; i++)
                {
                    if (faceFrameReaders[i].FaceFrameSource.IsTrackingIdValid)
                    {
                        quaternion2degree(faceFrameResults[i], out pitch, out yaw, out roll);
                        pitch = Math.Abs(pitch);
                        yaw = Math.Abs(yaw);
                        roll = Math.Abs(roll);
                        //ピッチ・ヨー・ロールを出力。
                        string PYRst = "," + pitch.ToString("F0") + "," + yaw.ToString("F0") + "," + roll.ToString("F0");
                        linevector = linevector + PYRst;
                        
                    }
                    
                }

            }

            catch (Exception ex)
            {

                linevector = linevector + zeroline;
            }
            //TBone.Text = a;
            if (linevector == present)
            {
                linevector = linevector + zeroline;
                TBone.Text = "Untracked"+"\t"+present;
            }

            else
            {
                TBone.Text = "Tracked" + "\t" + present;
            }

            sw.WriteLine(linevector);
            Nowpast = DateTime.Now;
        }

        //FaceFrameResultを受け取って角度を計算し、1つ前のコマとの角度の差分を返す関数
        void quaternion2degree(FaceFrameResult faceResult, out double pitch, out double yaw, out double roll)
        {
            Vector4 quaternion = faceResult.FaceRotationQuaternion;
            double w = quaternion.W;
            double x = quaternion.X;
            double y = quaternion.Y;
            double z = quaternion.Z;
            PITCH = (double)(Math.Atan2(2 * (y * z + w * x), w * w - x * x - y * y + z * z) / Math.PI * 180.0);
            YAW = (double)(Math.Asin(2 * (w * y - x * z)) / Math.PI * 180.0);
            ROLL = (double)(Math.Atan2(2 * (x * y + w * z), w * w + x * x - y * y - z * z) / Math.PI * 180.0);

            //1つ前のコマとの差分
            pitch = PITCH - opitch;
            yaw = YAW - oyaw;
            roll = ROLL - oroll;
            if (faceResult.FaceRotationQuaternion == null)
            {
                pitch = 3.0;
                yaw = 3.0;
                roll = 3.0;
            }
        }

        //各々の関節に対してSpineBaseとの相対的な座標を得る。
        public double Distance(Body body, Joint joint)
        {
            string ThisJoint = joint.JointType.ToString();
            try
            {
                double[] coordinate = { joint.Position.X- body.Joints[JointType.SpineBase].Position.X,
                    joint.Position.Y-body.Joints[JointType.SpineBase].Position.Y,
                    joint.Position.Z-body.Joints[JointType.SpineBase].Position.Z};
                squaredistance = Math.Pow((coordinate[0] - JC[ThisJoint][0]), 2) + Math.Pow((coordinate[1] - JC[ThisJoint][1]), 2) + Math.Pow((coordinate[2] - JC[ThisJoint][2]), 2);
                distance = Math.Pow(squaredistance, 0.5) * 1000;
                JC[ThisJoint] = coordinate;
                return distance;
            }
            catch (Exception ex)
            {
                Dictionary<string, double[]> dict = new Dictionary<string, double[]>();
                double[] zero = new double[] { 0.0, 0.0, 0.0 };
                dict[ThisJoint] = zero;
                return 0;
            }
        }

        void InitializeFace()
        {
            FaceFrameFeatures faceFrameFeatures =
                    FaceFrameFeatures.BoundingBoxInColorSpace
                    | FaceFrameFeatures.PointsInColorSpace
                    | FaceFrameFeatures.RotationOrientation
                    | FaceFrameFeatures.FaceEngagement
                    | FaceFrameFeatures.Glasses
                    | FaceFrameFeatures.Happy
                    | FaceFrameFeatures.LeftEyeClosed
                    | FaceFrameFeatures.RightEyeClosed
                    | FaceFrameFeatures.LookingAway
                    | FaceFrameFeatures.MouthMoved
                    | FaceFrameFeatures.MouthOpen;
            faceFrameSources = new FaceFrameSource[bodyCount];
            faceFrameReaders = new FaceFrameReader[bodyCount];
            for (int i = 0; i < bodyCount; i++)
            {
                faceFrameSources[i] = new FaceFrameSource(kinect, 0, faceFrameFeatures);
                faceFrameReaders[i] = faceFrameSources[i].OpenReader();
                faceFrameReaders[i].FrameArrived += faceFrameReader_FrameArrived;
            }
            faceFrameResults = new FaceFrameResult[bodyCount];

        }
        void UpdateBodyFrame(BodyFrameArrivedEventArgs e)
        {
            using (var bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame == null)
                {
                    return;
                }
                bodyFrame.GetAndRefreshBodyData(bodies);
                for (int i = 0; i < bodyCount; i++)
                {
                    Body body = bodies[i];
                    if (!body.IsTracked)
                    {
                        continue;
                    }
                    ulong trackingId = body.TrackingId;
                    faceFrameReaders[i].FaceFrameSource.TrackingId = trackingId;
                }
            }
        }

        void faceFrameReader_FrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            UpdateFaceFrame(e);
        }

        void UpdateFaceFrame(FaceFrameArrivedEventArgs e)
        {
            using (FaceFrame faceFrame = e.FrameReference.AcquireFrame())
            {
                if (faceFrame == null)
                {
                    return;
                }
                bool tracked;
                tracked = faceFrame.IsTrackingIdValid;
                if (!tracked)
                {
                    return;
                }

                FaceFrameResult faceResult = faceFrame.FaceFrameResult;
                int index = GetFaceSourceIndex(faceFrame.FaceFrameSource);
                faceFrameResults[index] = faceResult;
            }
        }

        int GetFaceSourceIndex(FaceFrameSource source)
        {
            int index = -1;
            for (int i = 0; i < bodyCount; i++)
            {
                if (faceFrameSources[i] == source)
                {
                    index = i;
                    break;
                }
            }
            return index;
        }

        void bodyFrameReader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            UpdateBodyFrame(e);
            
        }

        void colorFrameReader_FrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            UpdateColorFrame(e);
            DrawColorFrame();
        }

        private void UpdateColorFrame(ColorFrameArrivedEventArgs e)
        {
            // カラーフレームを取得する
            using (var colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame == null)
                {
                    return;
                }

                // BGRAデータを取得する
                colorFrame.CopyConvertedFrameDataToArray(
                                            colorBuffer, colorFormat);
            }
        }

        private void DrawColorFrame()
        {
            // ビットマップにする
            colorBitmap.WritePixels(colorRect, colorBuffer,
                                            colorStride, 0);
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            for (int i = 0; i < bodyCount; i++)
            {
                if (faceFrameReaders[i] != null)
                {
                    faceFrameReaders[i].Dispose();
                    faceFrameReaders[i] = null;
                }
                if (faceFrameSources[i] != null)
                {
                    faceFrameSources[i].Dispose();
                    faceFrameSources[i] = null;
                }
            }
            if (bodyFrameReader != null)
            {
                bodyFrameReader.Dispose();
                bodyFrameReader = null;
            }
            if (kinect != null)
            {
                kinect.Close();
                kinect = null;
            }

            sw.Close();
        }
        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            try
            {
                kinect = KinectSensor.GetDefault();
                if (kinect == null)
                {
                    throw new Exception("Kinectを開けません");
                }
                kinect.Open();
                FrameDescription frameDescription = kinect.ColorFrameSource.FrameDescription;

                bodyFrameReader = kinect.BodyFrameSource.OpenReader();
                bodyFrameReader.FrameArrived += bodyFrameReader_FrameArrived;
                bodyCount = kinect.BodyFrameSource.BodyCount;
                bodies = new Body[bodyCount];

                // カラー画像の情報を作成する(BGRAフォーマット)
                colorFrameDesc = kinect.ColorFrameSource.CreateFrameDescription(
                                                        colorFormat);

                // カラーリーダーを開く
                colorFrameReader = kinect.ColorFrameSource.OpenReader();
                colorFrameReader.FrameArrived += colorFrameReader_FrameArrived;

                // カラー用のビットマップを作成する
                colorBitmap = new WriteableBitmap(
                                    colorFrameDesc.Width, colorFrameDesc.Height,
                                    96, 96, PixelFormats.Bgra32, null);
                colorStride = colorFrameDesc.Width * (int)colorFrameDesc.BytesPerPixel;
                colorRect = new Int32Rect(0, 0,
                                    colorFrameDesc.Width, colorFrameDesc.Height);
                colorBuffer = new byte[colorStride * colorFrameDesc.Height];
                ImageColor.Source = colorBitmap;

                this._timer = new DispatcherTimer();
                this._timer.Interval = TimeSpan.FromMilliseconds(33);
                this._timer.Tick += _timer_Tick;

                this._timer.Start();
                this._startTime = DateTime.Now;
                double[] zero = new double[] { 0.0, 0.0, 0.0 };

                //関節の初期座標としてzeroを与える。
                JC["AnkleLeft"] = JC["AnkleRight"] = JC["ElbowLeft"] = JC["ElbowRight"] = JC["FootLeft"] = JC["FootRight"] = JC["HandLeft"] = JC["HandRight"] = JC["HandTipLeft"] = JC["HandTipRight"] = JC["Head"] = JC["HipLeft"] = JC["HipRight"] = JC["KneeLeft"] = JC["KneeRight"] = JC["Neck"] = JC["ShoulderLeft"] = JC["ShoulderRight"] = JC["SpineBase"] = JC["SpineMid"] = JC["SpineShoulder"] = JC["ThumbLeft"] = JC["ThumbRight"] = JC["WristLeft"] = JC["WristRight"] = zero;

                //関節の制限速度を定義
                JT["AnkleLeft"] = JT["AnkleRight"] = 40.0; JT["ElbowLeft"] = JT["ElbowRight"] = 80.0;
                JT["FootLeft"] = JT["FootRight"] = 40.0; JT["HandLeft"] = JT["HandRight"] = 200.0;
                JT["HandTipLeft"] = JT["HandTipRight"] = 200.0; JT["Head"] = 10.0; JT["HipLeft"] = JT["HipRight"] = 5.0;
                JT["KneeLeft"] = JT["KneeRight"] = 40.0; JT["Neck"] = 10.0; JT["ShoulderLeft"] = JT["ShoulderRight"] = 20.0;
                JT["SpineBase"] = 10.0; JT["SpineMid"] = 10.0; JT["SpineShoulder"] = 10.0;
                JT["ThumbLeft"] = JT["ThumbRight"] = 200.0; JT["WristLeft"] = JT["WristRight"] = 200.0;

                InitializeFace();
            }
            catch (Exception ex)
            {
                MessageBox.Show(ex.Message);
                Close();
            }
        }
    }
}