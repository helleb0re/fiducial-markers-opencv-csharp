using OpenCvSharp;
namespace comparing_fiducial_markers;

public class Camera
{
    // Фокусное расстояние объектива
    public double FocalLength { get; }

    // Размеры матрицы в мм
    public double SensorWidth { get; } // 54.12
    public double SensorHeight { get; } // 25.59
    
    // Размеры матрицы в пикселях
    public int SensorWidthInPixels { get; }
    public int SensorHeightInPixels { get; }
    
    // Внутренние параметры камеры
    public double[,] IntrinsicParams { get; private set; } = new double[3, 3];

    public double[] DistCoeffs { get; private set; } = new double[14];

    public double[,] RotMat { get; private set; } = {{1.0, 0, 0}, {0, -1.0, 0}, {0, 0, -1.0}};
    public double[] TransVect { get; private set; } = {0.0, 0.0, 1.0};
    
    public double[,] CoordOnScreen2D { get; private set; }
    
    
    // Конструктор для инициализации экземпляров класса камеры
    public Camera(double focalLength, double sensorWidth, double sensorHeight, 
        int sensorWidthInPixels, int sensorHeightInPixels)
    {
        FocalLength = focalLength;
        SensorWidth = sensorWidth;
        SensorHeight = sensorHeight;
        SensorWidthInPixels = sensorWidthInPixels;
        SensorHeightInPixels = sensorHeightInPixels;
        
        InitCameraMatrix();
    }
    // Конструктор для инициализации экземпляров класса камеры с параметрами дисторсии
    public Camera(double focalLength, double sensorWidth, double sensorHeight, 
        int sensorWidthInPixels, int sensorHeightInPixels, double[] distCoeffs) 
        : this(focalLength, sensorWidth, sensorHeight, sensorWidthInPixels, sensorHeightInPixels)
    {
        InitDistortionCoefficients(distCoeffs);
    }

    private void InitCameraMatrix()
    {
        IntrinsicParams[0, 0] = SensorWidthInPixels / SensorWidth * FocalLength;
        IntrinsicParams[0, 2] = SensorWidthInPixels / 2;
        IntrinsicParams[1, 1] = SensorWidthInPixels / SensorWidth * FocalLength;
        IntrinsicParams[1, 2] = SensorHeightInPixels / 2;
        IntrinsicParams[2, 2] = 1;
    }

    private void InitDistortionCoefficients(double[] distArr)
    {
        for (int i = 0; i < distArr.Length; i++) DistCoeffs[i] = distArr[i];
    }

    public void SetRotMat(double[,] r)
    {
        RotMat = r;
    }

    public void SetRotMat(double[] eulerVec)
    {
        Mat tmp = ExtMathFuncs.Euler2Rotm(new Mat(1, 3, MatType.CV_64F, 
            new []
            {
                eulerVec[0] / 180 * Math.PI,
                eulerVec[1] / 180 * Math.PI,
                eulerVec[2] / 180 * Math.PI,
            }));

        for (int i = 0; i < tmp.Rows; i++)
        {
            for (int j = 0; j < tmp.Cols; j++)
            {
                RotMat[i, j] = tmp.At<double>(i, j);
            }
        }
    }

    public void SetTransVec(double[] t)
    {
        TransVect = t;
    }
    
    // расчет матрицы координат точек проекции на матрицу камеры
    public Point2d[] CalcCoordOnScreen2D(double[,] coord3D)
    {
        Mat p = new Mat(4, 3, MatType.CV_64F, coord3D);
        Mat r = new Mat(3, 3, MatType.CV_64F, RotMat);
        Mat t = new Mat(3, 1, MatType.CV_64F, TransVect);
        Mat k = new Mat(3, 3, MatType.CV_64F, IntrinsicParams);
        Mat d = new Mat(1, 14, MatType.CV_64F, DistCoeffs);
        Mat c2 = new Mat();
        
        Mat tmp = new Mat(3, 4, MatType.CV_64F);
        Cv2.HConcat(r, t, tmp);

        Mat tmp1 = new Mat(4, 4, MatType.CV_64F);
        Cv2.VConcat(p.T(), Mat.Ones(MatType.CV_64F, new[] {1, 4}), tmp1);
        Mat pp = tmp * tmp1;
        
        ShowMatrix(p);
        ShowMatrix(pp);

        Cv2.ProjectPoints(p, r, t, k, d, c2);
        c2 = c2.Reshape(1);

        Point2d[] cc = new Point2d[4];
        cc[0].X = c2.At<double>(0, 0);
        cc[0].Y = c2.At<double>(0, 1);
        cc[1].X = c2.At<double>(1, 0);
        cc[1].Y = c2.At<double>(1, 1);
        cc[2].X = c2.At<double>(2, 0);
        cc[2].Y = c2.At<double>(2, 1);
        cc[3].X = c2.At<double>(3, 0);
        cc[3].Y = c2.At<double>(3, 1);


        return cc;
    }

    public Point3d[] CalcPoint3ds(Point2f[] coord2d)
    {
        Mat matCoord2d = new Mat(coord2d.Length, 3, MatType.CV_64F);
        for (int i = 0; i < coord2d.Length; i++)
        {
            matCoord2d.At<double>(i, 0) = coord2d[i].X;
            matCoord2d.At<double>(i, 1) = coord2d[i].Y;
            matCoord2d.At<double>(i, 2) = 1.0;
        }
        Mat r = new Mat(3, 3, MatType.CV_64F, RotMat);
        Mat t = new Mat(3, 1, MatType.CV_64F, TransVect);
        Mat k = new Mat(3, 3, MatType.CV_64F, IntrinsicParams);
        
        Mat zc = k.Inv() * matCoord2d.T();
        Mat tmp1 = Mat.Ones(new Size(coord2d.Length, 1), MatType.CV_64F);
        Cv2.VConcat(zc, tmp1, zc);

        Mat rt = new Mat(3, 4, MatType.CV_64F);
        Cv2.HConcat(r, t, rt);
        Mat tmp = new Mat(1, 4, MatType.CV_64F, new[] {0.0, 0.0, 0.0, 1.0});
        Cv2.VConcat(rt, tmp, rt);

        Mat zw = rt.Inv() * zc;
        
        // zw = zw.T();
        // for (int i = 0; i < zw.Rows; i++)
        // {
        //     Cv2.Divide(zw.Row(i), zw.At<double>(i, 3), zw.Row(i));
        // }
        //
        // zw = zw.ColRange(0, zw.Cols - 1);
        
        Cv2.ConvertPointsFromHomogeneous(zw.T(), zw);
        zw = zw.Reshape(1);

        Point3d[] cc = new Point3d[coord2d.Length];
        cc[0].X = zw.At<double>(0, 0);
        cc[0].Y = zw.At<double>(0, 1);
        cc[0].Z = zw.At<double>(0, 2);
        cc[1].X = zw.At<double>(1, 0);
        cc[1].Y = zw.At<double>(1, 1);
        cc[1].Z = zw.At<double>(1, 2);
        cc[2].X = zw.At<double>(2, 0);
        cc[2].Y = zw.At<double>(2, 1);
        cc[2].Z = zw.At<double>(2, 2);
        cc[3].X = zw.At<double>(3, 0);
        cc[3].Y = zw.At<double>(3, 1);
        cc[3].Z = zw.At<double>(3, 2);

        return cc;
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
}