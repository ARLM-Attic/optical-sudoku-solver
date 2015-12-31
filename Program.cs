#define Tst1
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.InteropServices;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;


namespace OpticalSudokuSolver
{
    public static class MatExt
    {
        public static double GetDoubleValue(this Mat mat, int row, int col)
        {
            var value = new double[1];
            Marshal.Copy(mat.DataPointer + (row * mat.Cols + col) * mat.ElementSize, value, 0, 1);
            return value[0];
        }
        public static void SetDoubleValue(this Mat mat, int row, int col, double value)
        {
            var target = new[] { value };
            Marshal.Copy(target, 0, mat.DataPointer + (row * mat.Cols + col) * mat.ElementSize, 1);
        }
        public static void drawLine(this Mat img, PointF line, MCvScalar rgb)
        {
	        if(line.Y!=0)
	        {
		        float m = (float)(-1.0/Math.Tan(line.Y));
		        float c = (float)(line.X/Math.Sin(line.Y));
                CvInvoke.Line(img, new Point(0, (int)c), new Point(img.Size.Width, (int)(m*img.Size.Width+c)), rgb);
	        }
	        else
	        {
                CvInvoke.Line(img, new Point((int)line.X, 0), new Point((int)line.X, img.Size.Height), rgb);
	        }
        }
        public static PointF[] mergeRelatedLines(this Mat img, PointF[] lines, float maxDeltaP, float maxDeltaTheta)
        {
            List<PointF> lstLines = new List<PointF>();
            List<int> lstLinesCnt = new List<int>();
            int n = lines.Length;
            for (int i = 0; i < n; i++)
            {
                if (lines[i].X < 0)
                {
                    lines[i].X = -lines[i].X;
                    lines[i].Y = (float)Math.PI + lines[i].Y;
                }
            }
            for (int i = 0; i < n; i++)
            {
                int idx = lstLines.FindIndex(ln =>
                {
                    if (Math.Abs(ln.X - lines[i].X) < maxDeltaP)
                    {
                        float dTheta = Math.Abs(ln.Y - lines[i].Y);
                        if (dTheta > (float)Math.PI)
                        {
                            dTheta = (float)(2 * Math.PI) - dTheta;
                        }
                        return dTheta < maxDeltaTheta;
                    }
                    return false;
                });
                if(idx < 0)
                {
                    lstLines.Add(lines[i]);
                    lstLinesCnt.Add(1);
                }
                else
                {
                    int nInLst = lstLinesCnt[idx];
                    float p = (lstLines[idx].X * nInLst + lines[i].X) / (nInLst + 1);
                    float theta = lstLines[idx].Y;
                    if (Math.Abs(theta - lines[i].Y) > maxDeltaTheta)
                    {
                        if (theta < lines[i].Y)
                        {
                            theta = (nInLst * theta + lines[i].Y - (float)(2 * Math.PI)) / (nInLst + 1);
                        }
                        else
                        {
                            theta = (nInLst * theta + lines[i].Y + (float)(2 * Math.PI)) / (nInLst + 1);
                        }
                    }
                    else 
                    {
                        theta = (nInLst * theta + lines[i].Y) / (nInLst + 1);
                    }
                    if(theta < 0)
                    {
                        theta += (float)(2 * Math.PI);
                    }
                    else if(theta > (float)(2 * Math.PI))
                    {
                        theta -= (float)(2 * Math.PI);
                    }
                    lstLinesCnt[idx]++;
                    lstLines[idx] = new PointF(p, theta);
                }
            }
            return lstLines.ToArray();
        }
        // Return first 2 cross line group
        public static List<List<int>> classifyLines(this Mat img, PointF[] lines, float maxDeltaTheta)
        {
            int n = lines.Length;
            List<int> idxLst = new List<int>();
            for(int i = 0; i < n; i++)
            {
                idxLst.Add(i);
            }
            List<float> thetas = new List<float>();
            List<List<int>> ret = new List<List<int>>();
            while(idxLst.Count > 0)
            {
                List<int> curRet = new List<int>();
                int idx = idxLst[idxLst.Count - 1];
                idxLst.RemoveAt(idxLst.Count - 1);
                curRet.Add(idx);
                thetas.Add(lines[idx].Y);

                float theta0 = lines[idxLst[0]].Y;
                for(int i = idxLst.Count-1; i >= 0; i--)
                {
                    idx = idxLst[i];
                    float theta1 = lines[idx].Y;
                    float dTheta = Math.Abs(theta0 - theta1);
                    if(dTheta > (float)Math.PI)
                    {
                        dTheta = (float)(2*Math.PI) - dTheta;
                    }
                    if(dTheta < maxDeltaTheta)
                    {
                        idxLst.RemoveAt(i);
                        curRet.Add(idx);
                    }
                }
                ret.Add(curRet);
            }
            for (int i = 0; i < thetas.Count; i++)
            {
                int idx = thetas.FindIndex(th =>
                {
                    float dTheta = Math.Abs(th - thetas[i]);
                    return Math.Abs(dTheta - (float)(0.5 * Math.PI)) < maxDeltaTheta || Math.Abs(dTheta - (float)(1.5 * Math.PI)) < maxDeltaTheta;
                });
                if(idx >= 0)
                {
                    List<int> ret0 = ret[i];
                    List<int> ret1 = ret[idx];
                    ret.Clear();
                    ret.Add(ret0);
                    ret.Add(ret1);
                    break;
                }
            }
            return ret;
        }
        public static List<Point> calculateIntersectionPoints(this Mat img, PointF[] lines, List<int> grp0, List<int> grp1)
        {
            int idxMax0 = grp0[0];
            int idxMin0 = grp0[0];
            for (int i = 1; i < grp0.Count; i++)
            {
                if(lines[grp0[i]].X > lines[idxMax0].X)
                {
                    idxMax0 = grp0[i];
                }
                if(lines[grp0[i]].X < lines[idxMin0].X)
                {
                    idxMin0 = grp0[i];
                }
            }

            int idxMax1 = grp1[0];
            int idxMin1 = grp1[0];
            for (int i = 1; i < grp1.Count; i++)
            {
                if (lines[grp1[i]].X > lines[idxMax1].X)
                {
                    idxMax1 = grp1[i];
                }
                if (lines[grp1[i]].X < lines[idxMin1].X)
                {
                    idxMin1 = grp1[i];
                }
            }

            //Point p0 = 
            return null;
        }
    }

   class Program
   {
#if Tst1
        static void Main(string[] args)
        {
            string strSudokuPic = "../tstData/sudoku-original.jpg";
            Mat sudoku = new Mat(strSudokuPic, LoadImageType.Grayscale);
            byte[] data = new byte[sudoku.Size.Width * sudoku.Size.Height];
            GCHandle handle = GCHandle.Alloc(data, GCHandleType.Pinned);
            Mat outerBox = new Mat(sudoku.Size, DepthType.Cv8U, 1, handle.AddrOfPinnedObject(), sudoku.Size.Width);
            //Mat[] chs = img.Split();
            //CvInvoke.Add(chs[0], chs[1], chs[1]);
            //CvInvoke.Subtract(chs[2], chs[1], chs[2]);
            CvInvoke.GaussianBlur(sudoku, outerBox, new System.Drawing.Size(11, 11), 0);
            CvInvoke.AdaptiveThreshold(outerBox, outerBox, 255, AdaptiveThresholdType.MeanC, ThresholdType.BinaryInv, 5, 2);
            byte[,] dat = new byte[3,3]{{0,1,0,},{1,1,1,},{0,1,0}};
            Mat kernel = new Mat();
            kernel.SetTo(dat);
            CvInvoke.Dilate(outerBox, outerBox, kernel, new Point(), 1, BorderType.Default, new MCvScalar());
            
            int max = -1;
            Point maxPt = new Point();
            Rectangle rect;     
            int idx = 0;
            for (int y = 0; y < outerBox.Size.Height; y++)
            {
                for (int x = 0; x < outerBox.Size.Width; x++)
                {
                    if (data[idx++] >= 128)
                    {
                        int area = CvInvoke.FloodFill(outerBox, null, new Point(x, y), new MCvScalar(64, 0, 0), out rect, new MCvScalar(), new MCvScalar());
                        if (area > max)
                        {
                            maxPt.X = x;
                            maxPt.Y = y;
                            max = area;
                        }
                    }
                }
            }
            CvInvoke.FloodFill(outerBox, null, maxPt, new MCvScalar(255, 255, 255), out rect, new MCvScalar(), new MCvScalar());
            idx = 0;
            for (int y = 0; y < outerBox.Size.Height; y++)
            {
                for (int x = 0; x < outerBox.Size.Width; x++)
                {
                    if (data[idx++] == 64 && x != maxPt.X && y != maxPt.Y)
                    {
                        int area = CvInvoke.FloodFill(outerBox, null, new Point(x, y), new MCvScalar(0, 0, 0), out rect, new MCvScalar(), new MCvScalar());
                    }
                }
                //printf("Current row: %d\n", y);
            }
            CvInvoke.Erode(outerBox, outerBox, kernel, new Point(), 1, BorderType.Default, new MCvScalar());

            Emgu.CV.Util.VectorOfPointF vp = new Emgu.CV.Util.VectorOfPointF();
            CvInvoke.HoughLines(outerBox, vp, 1, Math.PI/180, 200);
            //CvInvoke.HoughLinesP(outerBox, vp, 1, Math.PI / 180, 80, outerBox.Size.Width / 2.0, 50);
            PointF[] lines = vp.ToArray();
            lines = sudoku.mergeRelatedLines(lines, 15, (float)(15 * Math.PI / 180));
            List<List<int>> ret = sudoku.classifyLines(lines, (float)(15 * Math.PI / 180));
            int cntLines = Math.Min(99, lines.Length);
            for (int i = 0; i < cntLines; i++)
            {
                outerBox.drawLine(lines[i], new MCvScalar(128,0,0));
            }

            CvInvoke.Imshow("Ori", sudoku);
            CvInvoke.Imshow("out", outerBox);
            CvInvoke.WaitKey(0);
            handle.Free();
            CvInvoke.DestroyAllWindows();
            return;
        }
#elif Tst0
       static void Main(string[] args)
       {
           Image<Bgr, byte> img = new Image<Bgr, byte>("c:/ocr/thresholding_example.jpg");
           Image<Gray, byte>[] chs = img.Split();
           chs[0] = chs[0].Add(chs[1]);
           chs[2] = chs[2].Sub(chs[0]).ThresholdBinary(new Gray(20), new Gray(255));
           CvInvoke.Imshow("Ori", img);
           CvInvoke.Imshow("Red", chs[2]);
           CvInvoke.WaitKey(0);
           CvInvoke.DestroyAllWindows();
           return;
       }
#else
      static void Main(string[] args)
      {
         String win1 = "Test Window"; //The name of the window
         CvInvoke.NamedWindow(win1); //Create the window using the specific name

         Mat img = new Mat(200, 400, DepthType.Cv8U, 3); //Create a 3 channel image of 400x200
         img.SetTo(new Bgr(255, 0, 0).MCvScalar); // set it to Blue color

         //Draw "Hello, world." on the image using the specific font
         CvInvoke.PutText(
            img, 
            "Hello, world", 
            new Point(10, 80), 
            FontFace.HersheyComplex, 
            1.0, 
            new Bgr(0, 255, 0).MCvScalar);
         

         CvInvoke.Imshow(win1, img); //Show the image
         CvInvoke.WaitKey(0);  //Wait for the key pressing event
         CvInvoke.DestroyWindow(win1); //Destroy the window if key is pressed
      }
#endif
   }
}