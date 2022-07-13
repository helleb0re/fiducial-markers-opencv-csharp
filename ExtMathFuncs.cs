using OpenCvSharp;
using static System.Math;

namespace comparing_fiducial_markers;

public class ExtMathFuncs
{
    public static Mat Rotm2Euler(Mat r)
    {
        double sy = Sqrt(r.At<double>(0, 0) * r.At<double>(0, 0) + r.At<double>(1, 0) * r.At<double>(1, 0));
        bool singular = sy < 1e-6;
            
        Mat eulerVec = Mat.Zeros(MatType.CV_64F, new[] {1, 3});
        if (!singular)
        {
            eulerVec.At<double>(0, 0) = Atan2(r.At<double>(1, 0), r.At<double>(0, 0));
            eulerVec.At<double>(0, 1) = Atan2(-r.At<double>(2, 0), sy);
            eulerVec.At<double>(0, 2) = Atan2(r.At<double>(2, 1), r.At<double>(2, 2));
        }
        else
        {
            eulerVec.At<double>(0, 0) = 0;
            eulerVec.At<double>(0, 1) = Atan2(-r.At<double>(2, 0), sy);
            eulerVec.At<double>(0, 2) = Atan2(-r.At<double>(1, 2), r.At<double>(1, 1));
        }
            
        return eulerVec;
    }
    
    public static double[] Rotm2Euler(double[,] r)
    {
        double sy = Sqrt(r[0, 0] * r[0, 0] + r[1, 0] * r[1, 0]);
        bool singular = sy < 1e-6;
        
        double[] eulerVec = new double[3];
        if (!singular)
        {
            eulerVec[0] = Atan2(r[1, 0], r[0, 0]);
            eulerVec[1] = Atan2(-r[2, 0], sy);
            eulerVec[2] = Atan2(r[2, 1], r[2, 2]);
        }
        else
        {
            eulerVec[0] = 0;
            eulerVec[1] = Atan2(-r[2, 0], sy);
            eulerVec[2] = Atan2(-r[1, 2], r[1, 1]);
        }

        return eulerVec;
    }

    public static Mat Euler2Rotm(Mat eulerZYX)
    {
        Mat rx = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {1, 0, 0},
            {0, Cos(eulerZYX.At<double>(0, 2)), -Sin(eulerZYX.At<double>(0, 2))},
            {0, Sin(eulerZYX.At<double>(0, 2)), Cos(eulerZYX.At<double>(0, 2))},
        });
        Mat ry = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {Cos(eulerZYX.At<double>(0, 1)), 0, Sin(eulerZYX.At<double>(0, 1))},
            {0, 1, 0},
            {-Sin(eulerZYX.At<double>(0, 1)), 0, Cos(eulerZYX.At<double>(0, 1))},
        });
        Mat rz = new Mat(3, 3, MatType.CV_64F, new [,]
        {
            {Cos(eulerZYX.At<double>(0, 0)), -Sin(eulerZYX.At<double>(0, 0)), 0},
            {Sin(eulerZYX.At<double>(0, 0)), Cos(eulerZYX.At<double>(0, 0)), 0},
            {0, 0, 1},
        });
        
        Mat r = rz * ry * rx;
        return r;
    }


    public static double ReprojectionError(Mat coord2Dl1, Mat coord2Dr1, Mat coord2Dl2, Mat coord2Dr2) => 
        Sqrt(((Mat)(((Mat) (coord2Dl1.Col(0) - coord2Dl2.Col(0))).Pow(2) 
        + ((Mat) (coord2Dr1.Col(1) - coord2Dr2.Col(1))).Pow(2))).Sum()[0] / coord2Dl1.Rows);

    public static double StandardDeviation(Mat coord3D1, Mat coord3D2) => 
        Sqrt(((Mat) (coord3D1 - coord3D2)).Pow(2).Sum()[0] / coord3D1.Rows);
}