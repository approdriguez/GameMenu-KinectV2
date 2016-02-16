﻿//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.BodyBasics
{
    using System;
    using System.Collections.Generic;
    using System.ComponentModel;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;             //Nombre de espacios para Kinect
    using System.Media;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Radius of drawn hand circles
        /// </summary>
        private const double HandSize = 30;

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Thickness of clip edge rectangles
        /// </summary>
        private const double ClipBoundsThickness = 10;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush/"Cepillo" utilizado para dibujar los joints de los huesos inferidos.
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen/"Lápiz" utilizado para dibujar huesos (bones) que son inferidos
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage imageSource;

        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// List of colors for each body tracked
        /// </summary>
        private List<Pen> bodyColors;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;


        // VARIABLES PROPIAS, NO PERTENECIENTES A LA DEMO.

        /// <summary>
        /// Coordenadas (X,Y) de los joints
        /// </summary>
        Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();

        /// <summary>
        /// Struct para almacenar y utilizar de forma más comoda las coordenadas de los joints
        /// </summary>
        public struct BodyPartCoords
        {
            public double x, y, z;

            public BodyPartCoords(double x_n, double y_n, double z_n)
            {
                this.x = x_n;
                this.y = y_n;
                this.z = z_n;
            }
        }

        /// <summary>
        /// Variables para saber cuando hay contacto entre joints 
        /// </summary>
        private bool RHand_Head = false;
        private bool LHand_Head = false;
        private bool RHand_Hip = false;
        private bool LHand_Hip = false;
        private bool RHand_LHand = false;

        /// <summary>
        /// Variables booleanas que nos permiten saber cuando hemos hecho la postura1 o 2.
        /// </summary>
        private bool posturaInicial = false;
        private bool postura2 = false;
        private bool tocarYin = false;

        /// <summary>
        /// Variable para contar el tiempo en que se está en la postura correcta.
        /// </summary>
        private int contadorFrames1 = 0;
        private int contadorFrames2 = 0;
        private int contadorFramesMusica = 0;

        /// <summary>
        /// SoundPlayer gong y kungfu son 2 variables que nos permiten utilizar sonidos en la aplicación.
        /// </summary>
        private SoundPlayer gong = new SoundPlayer(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Music\gong.wav");
        private SoundPlayer kungfu = new SoundPlayer(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Music\kung.wav");

        ImageBrush dojo1 = new ImageBrush(new BitmapImage(new Uri(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Images\dogo.jpg")));
        ImageBrush dojo2 = new ImageBrush(new BitmapImage(new Uri(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Images\dogo2.jpg")));
        ImageBrush dojo3 = new ImageBrush(new BitmapImage(new Uri(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Images\dogo3.jpg")));
        ImageBrush dojo4 = new ImageBrush(new BitmapImage(new Uri(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Images\dogo4.jpg")));
        ImageBrush yinyangBrush = new ImageBrush(new BitmapImage(new Uri(@"C:\Users\Jesús\Documents\GitHub\Práctica 1 NPI\Images\yinyang.png")));

        private bool fase1 = true;
        private bool fase2 = false;
        private bool fase3 = false;
        private bool fase4 = false;
        private bool PaintStart = true;
        private bool NowPlaying = false;

        // FIN VARIABLES PROPIAS, NO PERTENECIENTES A LA DEMO.


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // Console Message

            // KinectSensor es una variable privada definida en la linea 88
            // inicializada a null cuyo valor actualizamos con GetDefault();
            this.kinectSensor = KinectSensor.GetDefault();

            // Inicialización del coordinate mapper.
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width + 150;
            this.displayHeight = frameDescription.Height;

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // Un hueso (bone) esta compuesto por dos joints (puntos).
            // Joint type es un enumerado con los diferentes tipos que componen el 
            // esqueleto, la información se encuentra en:
            // https://msdn.microsoft.com/en-us/library/microsoft.kinect.jointtype.aspx
            this.bones = new List<Tuple<JointType, JointType>>();

            // Torso
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Head, JointType.Neck));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.Neck, JointType.SpineShoulder));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.SpineMid));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineMid, JointType.SpineBase));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineShoulder, JointType.ShoulderLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.SpineBase, JointType.HipLeft));

            // Brazo Derecho
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderRight, JointType.ElbowRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Brazo Izquierdo
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ShoulderLeft, JointType.ElbowLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Pierna derecha
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipRight, JointType.KneeRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeRight, JointType.AnkleRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleRight, JointType.FootRight));

            // Pierna izquierda
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HipLeft, JointType.KneeLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.KneeLeft, JointType.AnkleLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.AnkleLeft, JointType.FootLeft));

            // Para cada indice añade un color para el cuerpo
            this.bodyColors = new List<Pen>();

            // Pen(Brush,Double) Inicializa el objeto Pen con un Brush y un grosor (double).
            this.bodyColors.Add(new Pen(Brushes.Red, 6));    //Rojo
            this.bodyColors.Add(new Pen(Brushes.Orange, 6)); //Naranja
            this.bodyColors.Add(new Pen(Brushes.Green, 6));  //Verde
            this.bodyColors.Add(new Pen(Brushes.Blue, 6));   //Azul
            this.bodyColors.Add(new Pen(Brushes.Indigo, 6)); //Indigo (morado)
            this.bodyColors.Add(new Pen(Brushes.Violet, 6)); //Violeta

            // Establece el notificador de evento de cuando deja de estar disponible el sensor.
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // Abre el sensor
            this.kinectSensor.Open();

            // Establece el texto de estado
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // Crea el drawingGroup que se usará para dibujar-
            this.drawingGroup = new DrawingGroup();

            // Crea una fuente en el caso de que queramos utilizar el control de imagen.
            this.imageSource = new DrawingImage(this.drawingGroup);

            // Usa el objeto window como el view model.
            this.DataContext = this;

            // Inicializa los controles de la ventana.
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.imageSource;
            }
        }

        /// <summary>
        /// Obtiene o cambia el texto de estado a mostrar.
        /// Gets or sets the current status text to display.
        /// </summary>
        public string StatusText
        {
            //Obtener
            get
            {
                return this.statusText;
            }

            //Establecer
            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // Notifica cualquier conjunto de elementos a los que se les ha cambiado el texto.
                    // notify any bound elements that the text has changed.
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Ejecuta el comienzo de las tareas.
        /// Execute start up tasks.
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                this.bodyFrameReader.FrameArrived += this.Reader_FrameArrived;
            }
        }

        /// <summary>
        /// Ejecuta el cierre de tareas.
        /// Execute shutdown tasks.
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

        /// <summary>
        /// Maneja los datos de frames del cuerpo que llegan del sensor.
        /// Handles the body frame data arriving from the sensor.
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_FrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {

                if (bodyFrame != null)
                {
                    //Bodies es el array de cuerpo definido arriba.
                    if (this.bodies == null)
                    {
                        //Si no está definido el array creamos uno con el número de cuerpos que detectemos.
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.

                    // La primera vez que se llama a GetAndRefreshBodyData, Kinect asigna cada cuerpo en el array.
                    // Mientras estos objetos de body no esten establecidos como nulo en el array, estos serán
                    // reutilizados.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;

                }
            }

            if (dataReceived)
            {


                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    if (fase1)
                    {
                        dc.DrawRectangle(dojo1, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                    else if (fase2)
                    {
                        dc.DrawRectangle(dojo2, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                    else if (fase3)
                    {
                        dc.DrawRectangle(dojo3, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                    else if (fase4)
                    {
                        dc.DrawRectangle(dojo4, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                    //dc.DrawRectangle(Brushes.Red, null, new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
                    //dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    // Dibuja un fondo transparente para poner el tamaño de renderizado.
                    // Draw a transparent background to set the render size
                    //dc.DrawRectangle(Brushes.PaleTurquoise, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                    int penIndex = 0;
                    foreach (Body body in this.bodies)
                    {

                        Pen drawPen = this.bodyColors[penIndex++];

                        if (body.IsTracked)
                        {

                            this.DrawClippedEdges(body, dc);

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                            // Convierte los puntos "joints" a espacio de profundidad.
                            // convert the joint points to depth (display) space


                            foreach (JointType jointType in joints.Keys)
                            {
                                // A veces la profundidad(Z) de un joint inferido puede mostrarse negativa.
                                // 

                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                            }

                            this.DrawBody(joints, jointPoints, dc, drawPen);

                            Point posEsferaMusica = new Point(jointPoints[JointType.Head].X, (jointPoints[JointType.Head].Y) - 35.0);


                            // CODIGO PROPIO


                            // Dibujado del dojo inicial y al yinyang
                            if (PaintStart)
                            {
                                dc.DrawEllipse(yinyangBrush, null, posEsferaMusica, 20, 20);
                            }
                            if (PosturaStart())
                            {
                                tocarYin = true;
                            }
                            if (tocarYin)
                            {
                                contadorFramesMusica += 1;
                            }
                            if (!PosturaStart())
                            {
                                contadorFramesMusica = 0;
                            }
                            if (tocarYin)
                            {
                                if (!NowPlaying)
                                {
                                    NowPlaying = true;
                                    PaintStart = false;
                                    kungfu.Play();
                                    fase1 = false;
                                    fase2 = true;
                                }
                            }
                            //Comienzo de la primera postura
                            if (Postura1())
                            {
                                posturaInicial = true;
                            }
                            else if (!Postura1())
                            {
                                posturaInicial = false;
                                contadorFrames1 = 0;
                            }
                            if (posturaInicial)
                            {
                                contadorFrames1 += 1;
                                //dc.DrawEllipse(Brushes.Blue, null, jointPoints[JointType.HandRight], 20, 20);
                            }

                            if (contadorFrames1 >= 40)
                            {
                                fase2 = false;
                                fase3 = true;

                                contadorFrames2 += 1;
                            }
                            if (Postura2())
                            {
                                postura2 = true;
                            }
                            else if (!Postura2())
                            {
                                contadorFrames2 = 0;
                            }
                            if (postura2)
                            {
                                contadorFrames2 += 1;
                            }

                            if (contadorFrames2 >= 40)
                            {
                                fase3 = false;
                                fase4 = true;
                            }
                            if (RightHandOnHead())
                            {
                                RHand_Head = true;

                            }

                            if (PosturaFinal())
                            {
                                kungfu.Stop();
                            }
                            /// FIN 
                            /// 
                            /// CAMBIAR DINÁMICAMENTE EL FONDO DE LA VENTAN
                            /// 

                        }

                        // Previene dibujar fuera de la zona de renderizado.
                        // prevent drawing outside of our render area

                        this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    }
                }
            }
        }

        /// <summary>
        /// Dibuja un cuerpo.
        /// Draws a body.
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// <param name="drawingPen">specifies color to draw a specific body</param>
        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones / Dibuja los huesos
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Dibuja los joints.
            // Draw the joints.
            foreach (JointType jointType in joints.Keys)
            {

                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Dibuja un hueso del cuerpo (joint a joint).
        /// Draws one bone of a body (joint to joint).
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {

            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // Si no se pueden encontrar los joints, se sale.
            // If we can't find either of these joints, exit.
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked

            // Asumimos que todo los huesos dibujados son inferidos a no ser que ambas
            // joints hayan sido trackeados. 
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }



        /// <summary>
        /// Dibuja los indicadores que muestran los bordes 
        /// Draws indicators to show which edges are clipping body data
        /// </summary>
        /// <param name="body">body to draw clipping information for</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        private void DrawClippedEdges(Body body, DrawingContext drawingContext)
        {
            FrameEdges clippedEdges = body.ClippedEdges;

            if (clippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, this.displayHeight - ClipBoundsThickness, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, this.displayWidth, ClipBoundsThickness));
            }

            if (clippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, this.displayHeight));
            }

            if (clippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(this.displayWidth - ClipBoundsThickness, 0, ClipBoundsThickness, this.displayHeight));
            }
        }

        /// <summary>
        /// Maneja los eventos que hacen que los sensores dejen de estar disponibles (Por ej. pausado, cerrado, desenchufado).
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // Cuando haya error, pone el texto de estado.
            // on failure, set the status text.
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }


        //METODOS AÑADIDOS AL CÓDIGO DEL EJEMPLO

        /// <summary>
        /// Método que devuelve true si la mano derecha esta tocando la cabeza y false en caso contrario.
        /// </summary>
        private bool RightHandOnHead()
        {
            double distance = 0;
            BodyPartCoords RHand = new BodyPartCoords(jointPoints[JointType.HandRight].X, jointPoints[JointType.HandRight].Y, 0.0);
            BodyPartCoords Head = new BodyPartCoords(jointPoints[JointType.Head].X, jointPoints[JointType.Head].Y, 0.0);

            distance = (RHand.x - Head.x) + (RHand.y - Head.y);
            if (Math.Abs(distance) > 0.25f)
                return false;

            else
                return true;
        }

        /// <summary>
        /// Método que devuelve true si la mano izquierda esta tocando la cabeza y false en caso contrario.
        /// </summary>
        private bool LeftHandOnHead()
        {
            double distance = 0;
            BodyPartCoords LHand = new BodyPartCoords(jointPoints[JointType.HandLeft].X, jointPoints[JointType.HandLeft].Y, 0.0);
            BodyPartCoords Head = new BodyPartCoords(jointPoints[JointType.Head].X, jointPoints[JointType.Head].Y, 0.0);

            distance = (LHand.x - Head.x) + (LHand.y - Head.y);
            if (Math.Abs(distance) > 0.25f)
                return false;

            else
                return true;
        }

        /// <summary>
        /// Método que devuelve true si la mano izquierda esta tocando la cadera(por la parte izquierda) y false en caso contrario.
        /// </summary>
        /// <returns></returns>
        private bool LeftHandOnHip()
        {
            double distance = 0;
            BodyPartCoords LHand = new BodyPartCoords(jointPoints[JointType.HandLeft].X, jointPoints[JointType.HandLeft].Y, 0.0);
            BodyPartCoords Hip = new BodyPartCoords(jointPoints[JointType.HipLeft].X, jointPoints[JointType.HipLeft].Y, 0.0);

            distance = (LHand.x - Hip.x) + (LHand.y - Hip.y);
            if (Math.Abs(distance) > 0.25f)
                return false;

            else
                return true;
        }
        /// <summary>
        /// Método que devuelve true si la mano derecha esta tocando la cadera(por la parte derecha) y false en caso contrario.
        /// </summary>
        /// <returns></returns>
        private bool RightHandOnHip()
        {
            double distance = 0;
            BodyPartCoords RHand = new BodyPartCoords(jointPoints[JointType.HandRight].X, jointPoints[JointType.HandRight].Y, 0.0);
            BodyPartCoords Hip = new BodyPartCoords(jointPoints[JointType.HipRight].X, jointPoints[JointType.HipRight].Y, 0.0);

            distance = (RHand.x - Hip.x) + (RHand.y - Hip.y);
            if (Math.Abs(distance) > 0.25f)
                return false;

            else
                return true;
        }


        /// <summary>
        /// Función para calcular el vector creado a partir de 2 puntos.
        /// Dados V1 y V2 se obtiene el vector V1V2 (v2.x-v1.x, v2.y-v1.y)
        /// </summary>
        /// <param name="p1_x"> Coordenada X del primer punto </param>
        /// <param name="p1_y"> Coordenada Y del primer punto</param>
        /// <param name="p2_x"> Coordenada X del segundo punto</param>
        /// <param name="p2_y"> Coordenada Y del segundo punto</param>
        /// <returns> Devuelve un vector con posición 0 = X y posición 1 = Y.</returns>

        private double[] GetVector(double p1_x, double p1_y, double p2_x, double p2_y)
        {
            double[] vect = new double[2];
            vect[0] = p1_x - p2_x;
            vect[1] = p1_y - p2_y;
            return vect;
        }

        /// <summary>
        /// Función que calculo el ángulo entre dos vectores.
        /// </summary>
        /// <param name="v1"> Vector 1</param>
        /// <param name="v2"> Vector 2</param>
        /// <returns> Angulo entre el vector 1 y el vector 2 </returns>
        private double GetAngulo(double[] v1, double[] v2)
        {
            double dividendo = ((v1[0] * v2[0]) + (v2[1] * v2[1]));
            double divisor = (Math.Sqrt((v1[0] * v1[0]) + (v1[1] * v1[1]))) * (Math.Sqrt((v2[0] * v2[0]) + (v2[1] * v2[1])));

            return Math.Acos(dividendo / divisor);
        }

        private bool touchAB(BodyPartCoords A, BodyPartCoords B)
        {
            bool tocado = false;
            double distancia = 0;

            distancia = (A.x - B.x) + (A.y - B.y);
            if (Math.Abs(distancia) > 1.20f)
                tocado = false;
            else
                tocado = true;

            return tocado;
        }

        //A debe estar a la altura de B
        private bool sameHeightAB(BodyPartCoords A, BodyPartCoords B)
        {
            bool same = false;
            double max, min;
            max = B.y + 10;
            min = B.y - 10;
            if (A.y > min && A.y < max)
                same = true;
            return same;
        }

        private bool Postura1()
        {
            bool detectado = false;
            //Partes necesitadas
            BodyPartCoords LeftFoot = new BodyPartCoords(jointPoints[JointType.FootLeft].X, jointPoints[JointType.FootLeft].Y, 0.0);
            BodyPartCoords LeftHip = new BodyPartCoords(jointPoints[JointType.HipLeft].X, jointPoints[JointType.HipLeft].Y, 0.0);
            BodyPartCoords LeftKnee = new BodyPartCoords(jointPoints[JointType.KneeLeft].X, jointPoints[JointType.KneeLeft].Y, 0.0);
            BodyPartCoords LeftAnkle = new BodyPartCoords(jointPoints[JointType.AnkleLeft].X, jointPoints[JointType.AnkleLeft].Y, 0.0);
            BodyPartCoords LeftHand = new BodyPartCoords(jointPoints[JointType.HandLeft].X, jointPoints[JointType.HandLeft].Y, 0.0);
            BodyPartCoords Hip = new BodyPartCoords(jointPoints[JointType.SpineBase].X, jointPoints[JointType.SpineBase].Y, 0.0);
            BodyPartCoords LeftElbow = new BodyPartCoords(jointPoints[JointType.ElbowLeft].X, jointPoints[JointType.ElbowLeft].Y, 0.0);
            BodyPartCoords RightElbow = new BodyPartCoords(jointPoints[JointType.ElbowRight].X, jointPoints[JointType.ElbowRight].Y, 0.0);
            BodyPartCoords MediaEspalda = new BodyPartCoords(jointPoints[JointType.SpineMid].X, jointPoints[JointType.SpineMid].Y - 10, 0.0);
            //Variables para los ángulos de la pose
            double rodillaIzq = 0;
            double brazoIzq = 0;
            double brazoDer = 0;

            /*
            //Ángulos
            //private double GetAngulo(double[] v1, double[] v2)
            //private double[] GetVector(double p1_x, double p1_y, double p2_x, double p2_y)

            //Vector rodilla-cadera
            double[] vknee_hip = GetVector(LeftKnee.x, LeftKnee.y, LeftHip.x,LeftHip.y );
            double[] vknee_ankle = GetVector(LeftKnee.x, LeftKnee.y, LeftAnkle.x, LeftAnkle.y);
            double knee_hip_ankle_degree = GetAngulo(vknee_hip, vknee_ankle);

            if (knee_hip_ankle_degree > 80 && knee_hip_ankle_degree < 110)
            //if(knee_hip_ankle_degree == 90)
                detectado = true;
        
            */

            if (sameHeightAB(LeftKnee, Hip) && sameHeightAB(LeftElbow, RightElbow))
                detectado = true;

            return detectado;
        }

        private bool Postura2()
        {
            bool detectado = false;

            //Partes necesitadas
            BodyPartCoords RightFoot = new BodyPartCoords(jointPoints[JointType.FootRight].X, jointPoints[JointType.FootRight].Y, 0.0);
            BodyPartCoords RightHip = new BodyPartCoords(jointPoints[JointType.HipRight].X, jointPoints[JointType.HipRight].Y, 0.0);
            BodyPartCoords RightKnee = new BodyPartCoords(jointPoints[JointType.KneeRight].X, jointPoints[JointType.KneeRight].Y, 0.0);
            BodyPartCoords RightAnkle = new BodyPartCoords(jointPoints[JointType.AnkleRight].X, jointPoints[JointType.AnkleRight].Y, 0.0);
            BodyPartCoords RightHand = new BodyPartCoords(jointPoints[JointType.HandRight].X, jointPoints[JointType.HandRight].Y, 0.0);
            BodyPartCoords Hip = new BodyPartCoords(jointPoints[JointType.SpineBase].X, jointPoints[JointType.SpineBase].Y, 0.0);
            BodyPartCoords RightElbow = new BodyPartCoords(jointPoints[JointType.ElbowRight].X, jointPoints[JointType.ElbowRight].Y, 0.0);
            BodyPartCoords LeftElbow = new BodyPartCoords(jointPoints[JointType.ElbowLeft].X, jointPoints[JointType.ElbowLeft].Y, 0.0);
            BodyPartCoords MediaEspalda = new BodyPartCoords(jointPoints[JointType.SpineMid].X, jointPoints[JointType.SpineMid].Y - 10, 0.0);

            /*
            //Variables para los ángulos de la pose
            double rodillaIzq = 0;
            double brazoIzq = 0;
            double brazoDer = 0;

            //Ángulos
            //private double GetAngulo(double[] v1, double[] v2)
            //private double[] GetVector(double p1_x, double p1_y, double p2_x, double p2_y)

            //Vector rodilla-cadera
            double[] vknee_hip = GetVector(RightKnee.x, RightKnee.y, RightHip.x,RightHip.y );
            double[] vknee_ankle = GetVector(RightKnee.x, RightKnee.y, RightAnkle.x, RightAnkle.y);
            double knee_hip_ankle_degree = GetAngulo(vknee_hip, vknee_ankle);

            if (knee_hip_ankle_degree > 80 && knee_hip_ankle_degree < 110)
            //if(knee_hip_ankle_degree == 90)
                detectado = true;

            */
            if (sameHeightAB(RightKnee, Hip) && sameHeightAB(LeftElbow, RightElbow))
                detectado = true;

            return detectado;
        }

        private bool PosturaStart()
        {
            BodyPartCoords LeftHand = new BodyPartCoords(jointPoints[JointType.HandLeft].X, jointPoints[JointType.HandLeft].Y, 0.0);
            BodyPartCoords Ball = new BodyPartCoords(jointPoints[JointType.Head].X, (jointPoints[JointType.Head].Y) - 35.0, 0.0);
            BodyPartCoords RightHand = new BodyPartCoords(jointPoints[JointType.HandRight].X, jointPoints[JointType.HandRight].Y, 0.0);
            bool aux1 = false;
            aux1 = touchAB(LeftHand, Ball);
            if (!aux1)
                aux1 = touchAB(RightHand, Ball);
            return aux1;

        }
        private bool PosturaFinal()
        {
            bool done = false;
            BodyPartCoords LeftHand = new BodyPartCoords(jointPoints[JointType.HandLeft].X, jointPoints[JointType.HandLeft].Y, 0.0);
            BodyPartCoords RightHand = new BodyPartCoords(jointPoints[JointType.HandRight].X, jointPoints[JointType.HandRight].Y, 0.0);
            BodyPartCoords Hip = new BodyPartCoords(jointPoints[JointType.SpineBase].X, jointPoints[JointType.SpineBase].Y, 0.0);
            if (touchAB(LeftHand, RightHand) && sameHeightAB(RightHand, Hip))
                done = true;
            return done;
        }
    }
}
