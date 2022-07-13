using System.Text;
using comparing_fiducial_markers;
using OpenCvSharp;


// Создаем экземпляр камеры
Camera camera = new Camera(19, 54.12, 25.59, 1920, 1080);
// Camera camera = new Camera(25, 8.44, 7.06, 2448, 2048);

// Создаем словарь с используемыми маркерами
// В нем хранится размер маркера, который используется при моделировании
float plateSize = 0.5f;
var aprilTag = new FiducialMarker("AprilTag16h5", 0.75f * plateSize);
var chiliTag = new FiducialMarker("ChiliTag", 0.7142857f * plateSize);
var sTag = new FiducialMarker("STagHD11", 0.8f * plateSize);
var arUco = new FiducialMarker("Aruco4x4", 0.777777777778f * plateSize);
var arToolKitPlus = new FiducialMarker("ARToolKitPlus", 0.8f * plateSize);

FiducialMarker[] markers = {aprilTag, chiliTag, sTag, arUco, arToolKitPlus};


Thread.CurrentThread.CurrentCulture = System.Globalization.CultureInfo.GetCultureInfo("en-US");

ProcessingData(Test.Rotate, markers, camera);
ProcessingData(Test.RotateX, markers, camera);
ProcessingData(Test.Translate, markers, camera);
ProcessingData(Test.RotateTranslate, markers, camera);
ProcessingData(Test.TranslateRotateY, markers, camera);
ProcessingData(Test.TranslateRotateX, markers, camera);


static void CountTrueSDError(Test test, FiducialMarker[] markers, Camera camera)
{
    double[] trueTransVec = {0.0, 0.0, 1.0};
    double[] trueRotsVec = {0.0, 0.0, -180.0}; // zyx

    foreach (FiducialMarker marker in markers)
    {
        int amountRotate = Directory.GetFiles($"data\\{marker.Name}\\{test.ToString()}", "*", 
            SearchOption.TopDirectoryOnly).Length;

        for (int i = 0; i < amountRotate; i++)
        {
            var sw = new StreamWriter($"output\\{marker.Name}\\{test.ToString()}\\{i}_err_px.txt");
            var sw1 = new StreamWriter($"output\\{marker.Name}\\{test.ToString()}\\{i}_true_px.txt");
            string[] lines = File.ReadAllLines($"data\\{marker.Name}\\{test.ToString()}\\output_{i}.txt");

            double[,] initCoords3D =
            {
                {-marker.Size / 2, marker.Size / 2, 0.01 / 2},
                {marker.Size / 2, marker.Size / 2, 0.01 / 2},
                {marker.Size / 2, -marker.Size / 2, 0.01 / 2},
                {-marker.Size / 2, -marker.Size / 2, 0.01 / 2},
            };
            Console.WriteLine(marker.Name);

            for (int j = 0; j < lines.Length; j++)
            {
                if (lines[j] == "not found") break;

                StringBuilder recordTextData = new StringBuilder();
                StringBuilder recordTextData1 = new StringBuilder();

                string[] s1 = lines[j].Split(";");
                Point2f[] corners = new Point2f[s1.Length - 1];
                for (int k = 0; k < s1.Length - 1; k++)
                {
                    string[] s2 = s1[k].Split(',');
                    corners[k].X = float.Parse(s2[0], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
                    corners[k].Y = float.Parse(s2[1], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
                }

                switch (test)
                {
                    case Test.Rotate:
                        trueRotsVec[1] = j;
                        break;
                    case Test.RotateX:
                        trueRotsVec[2] = 180 - j;
                        break;
                    case Test.Translate:
                        trueTransVec[2] = 1.0 + j * 0.25;
                        break;
                    case Test.RotateTranslate:
                        trueRotsVec[1] = i * 5;
                        trueTransVec[2] = j;
                        break;
                    case Test.TranslateRotateY:
                        trueRotsVec[1] = j;
                        trueTransVec[2] = 6 + i * 0.25;
                        break;
                    case Test.TranslateRotateX:
                        trueRotsVec[2] = 180 - j;
                        trueTransVec[2] = 1 + i * 0.25;
                        break;
                }
                
                camera.SetRotMat((double[])trueRotsVec.Clone());
                camera.SetTransVec((double[])trueTransVec.Clone());

                Point2d[] coords2d = camera.CalcCoordOnScreen2D(initCoords3D);
                for (int k = 0; k < coords2d.Length; k++)
                {
                    recordTextData1.Append($"{coords2d[k].X:F4},{coords2d[k].Y:F4};");
                }
                
                for (int k = 0; k < coords2d.Length; k++)
                {
                    recordTextData.Append(
                        $"{Math.Abs(coords2d[k].X - corners[k].X):F4},{Math.Abs(coords2d[k].Y - corners[k].Y):F4};");
                }
                
                sw.WriteLine(recordTextData);
                sw1.WriteLine(recordTextData1);
            }
            sw.Close();
            sw1.Close();
        }
    }
}

static void CountReprojectionErr(Test test, FiducialMarker[] markers, Camera camera)
{
    double[] trueTransVec = {0.0, 0.0, 1.0};
    double[] trueRotsVec = {0.0, 0.0, -180.0}; // zyx

    foreach (FiducialMarker marker in markers)
    {
        int amountRotate = Directory.GetFiles($"data\\{marker.Name}\\{test.ToString()}", "*", 
            SearchOption.TopDirectoryOnly).Length;

        double[,] initCoords3D =
        {
            {-marker.Size / 2, marker.Size / 2, 0},
            {marker.Size / 2, marker.Size / 2, 0},
            {marker.Size / 2, -marker.Size / 2, 0},
            {-marker.Size / 2, -marker.Size / 2, 0},
        };

        Console.WriteLine(marker.Name);
        for (int i = 0; i < amountRotate; i++)
        {
            var sw = new StreamWriter($"output\\{marker.Name}\\{test.ToString()}\\{i}_reprojection_err.txt");
            string[] lines = File.ReadAllLines($"data\\{marker.Name}\\{test.ToString()}\\output_{i}.txt");

            for (int j = 0; j < lines.Length; j++)
            {
                if (lines[j] == "not found") break;

                StringBuilder recordTextData = new StringBuilder();

                string[] s1 = lines[j].Split(";");
                Point2f[] corners = new Point2f[s1.Length - 1];
                for (int k = 0; k < s1.Length - 1; k++)
                {
                    string[] s2 = s1[k].Split(',');
                    corners[k].X = float.Parse(s2[0], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
                    corners[k].Y = float.Parse(s2[1], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
                }

                switch (test)
                {
                    case Test.Rotate:
                        trueRotsVec[1] = j;
                        
                        break;
                    case Test.Translate:
                        trueTransVec[2] = 1.0 + j * 0.25;

                        break;
                    case Test.RotateTranslate:
                        trueRotsVec[1] = i * 5;
                        trueTransVec[2] = j;
                        break;
                    case Test.TranslateRotateY:
                        trueRotsVec[1] = j;
                        trueTransVec[2] = 1 + i * 0.25;
                        break;
                    case Test.TranslateRotateX:
                        trueRotsVec[2] = 180 - j;
                        trueTransVec[2] = 1 + i * 0.25;
                        break;
                }
                                
                (double[] estTransVec, double[] estRotsVec) = DetectMarkers(lines[j], camera, marker.Size);
                camera.SetRotMat((double[])estRotsVec.Clone());
                camera.SetTransVec((double[])estTransVec.Clone());

                Point2d[] coords2d = camera.CalcCoordOnScreen2D(initCoords3D);
                
                for (int k = 0; k < coords2d.Length; k++)
                {
                    recordTextData.Append(
                        $"{Math.Abs(coords2d[k].X - corners[k].X):F5},{Math.Abs(coords2d[k].Y - corners[k].Y):F5};");
                }
                
                sw.WriteLine(recordTextData);
            }
            
            sw.Close();
        }
    }
}

static void ProcessingData(Test test, FiducialMarker[] markers, Camera camera)
{
    double[] trueTransVec = {0.0, 0.0, 1.0};
    double[] trueRotsVec = {0.0, 0.0, 180.0}; // zyx

    foreach (FiducialMarker marker in markers)
    {
        int amountRotate = Directory.GetFiles($"data\\{marker.Name}\\{test.ToString()}", "*", 
            SearchOption.TopDirectoryOnly).Length;
        marker.TestResults.Add(test, new Positions());
        
        marker.TestResults[test].TruePose = new Dictionary<int, List<Coordinates>>();
        marker.TestResults[test].EstimatedPose = new Dictionary<int, List<Coordinates>>();
        for (int i = 0; i < amountRotate; i++)
        {
            marker.TestResults[test].TruePose.Add(i, new List<Coordinates>());
            marker.TestResults[test].EstimatedPose.Add(i, new List<Coordinates>());
            Console.WriteLine(marker.Name);
            var sw = new StreamWriter($"output\\{marker.Name}\\{test.ToString()}\\{i}.txt");
            var sw1 = new StreamWriter($"output\\{marker.Name}\\{test.ToString()}\\{i}_err.txt");
            string[] lines = File.ReadAllLines($"data\\{marker.Name}\\{test.ToString()}\\output_{i}.txt");
            
            for (int j = 0; j < lines.Length; j++)
            {
                if (lines[j] == "not found") break;

                StringBuilder recordTextData = new StringBuilder();
                
                switch (test)
                {
                    case Test.Rotate:
                        trueRotsVec[1] = j;
                        break;
                    case Test.RotateX:
                        trueRotsVec[2] = 180 - j;
                        break;
                    case Test.Translate:
                        trueTransVec[2] = 1.0 + j * 0.25;
                        break;
                    case Test.RotateTranslate:
                        trueRotsVec[1] = i * 5;
                        trueTransVec[2] = j;
                        break;
                    case Test.TranslateRotateY:
                        trueRotsVec[1] = j;
                        trueTransVec[2] = 1 + i * 0.25;
                        break;
                    case Test.TranslateRotateX:
                        trueRotsVec[2] = 180 - j;
                        trueTransVec[2] = 1 + i * 0.25;
                        break;
                }
                
                marker.TestResults[test].TruePose[i].Add(new Coordinates
                {
                    X = trueTransVec[0],
                    Y = trueTransVec[1],
                    Z = trueTransVec[2],
                    RX = trueRotsVec[2],
                    RY = trueRotsVec[1],
                    RZ = trueRotsVec[0],
                });

                (double[] estTransVec, double[] estRotsVec) = DetectMarkers(lines[j], camera, marker.Size);

                double errX = Math.Abs(estTransVec[0] - trueTransVec[0]);
                double errY = Math.Abs(estTransVec[1] - trueTransVec[1]);
                double errZ = Math.Abs(estTransVec[2] - trueTransVec[2]);

                double errRX = Math.Abs(Math.Abs(estRotsVec[2]) - Math.Abs(trueRotsVec[2]));
                double errRY = Math.Abs(Math.Abs(estRotsVec[1]) - Math.Abs(trueRotsVec[1]));
                double errRZ = Math.Abs(Math.Abs(estRotsVec[0]) - Math.Abs(trueRotsVec[0]));

                recordTextData.Append($"{estTransVec[0]:F4};{estTransVec[1]:F4};{estTransVec[2]:F4};");
                recordTextData.Append($"{estRotsVec[2]:F4};{estRotsVec[1]:F4};{estRotsVec[0]:F4};");
                
                marker.TestResults[test].EstimatedPose[i].Add(new Coordinates
                {
                    X = estTransVec[0],
                    Y = estTransVec[1],
                    Z = estTransVec[2],
                    RX = estRotsVec[2],
                    RY = estRotsVec[1],
                    RZ = estRotsVec[0],
                });

                sw.WriteLine(recordTextData);
                sw1.WriteLine($"{errX:F5};{errY:F5};{errZ:F5};{errRX:F5};{errRY:F5};{errRZ:F5};");
            }
            sw.Close();
            sw1.Close();
        }
    }
}

static (double[], double[]) DetectMarkers(string dataString, Camera camera, float markerSize)
{
    // Фрагмент программы для считывания координат углов маркера на изображения,
    // которые определяются с помощью алгоритма детектирования 
    string[] s1 = dataString.Split(";");
    Point2f[] corners = new Point2f[s1.Length - 1];
    for (int i = 0; i < s1.Length - 1; i++)
    {
        string[] s2 = s1[i].Split(',');
        corners[i].X = float.Parse(s2[0], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
        corners[i].Y = float.Parse(s2[1], System.Globalization.CultureInfo.InvariantCulture.NumberFormat);
    }

    // Рассчитываем положения углов маркера в мировой системе координат. Формулы (21)-(24)
    Point3f[] objPts =
    {
        new (-markerSize / 2,markerSize / 2,0),
        new (markerSize / 2,markerSize / 2,0),
        new (markerSize / 2,-markerSize / 2,0),
        new (-markerSize / 2,-markerSize / 2,0),
    };
    // Инициализация векторов вращения и перемещения
    double[] rvec = { 0, 0, 0 };
    double[] tvec = { 0, 0, 0 };
    // Сама функция определения положения. Формула (28)
    Cv2.SolvePnP(objPts, corners, camera.IntrinsicParams, camera.DistCoeffs, 
        ref rvec, ref tvec, flags: SolvePnPFlags.Iterative);
    
    // Перевод из вектора вращения из нотации Родригеса в углы Эйлера. Формулы (30)-(32)
    Cv2.Rodrigues(rvec, out var R, out var jacobian);
    double[] eulerVec = ExtMathFuncs.Rotm2Euler(R);
    
    // Перевод из радиан в градусы
    eulerVec[0] *= 180 / Math.PI;
    eulerVec[1] *= 180 / Math.PI;
    eulerVec[2] *= 180 / Math.PI;
    
    return (tvec, eulerVec);
}

static void ShowMatrix(Mat matrix)
{
    for (int i = 0; i < matrix.Rows; i++)
    {
        for (int j = 0; j < matrix.Cols; j++)
        {
            if (j + 1 == matrix.Cols)
                Console.WriteLine($" {matrix.At<double>(i, j):F5}");
            else
                Console.Write($" {matrix.At<double>(i, j):F5},");
        }
    }

    Console.WriteLine();
}