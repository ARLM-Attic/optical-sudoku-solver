#define DebugMat
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.InteropServices;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;


namespace OpticalSudokuSolver
{
    public struct MyMat
    {
        public Mat _mat;
        public byte[] _data;
        public GCHandle _handle;
        public MyMat(Size sz)
        {
            _data = new byte[sz.Width * sz.Height];
            _handle = GCHandle.Alloc(_data, GCHandleType.Pinned);
            _mat = new Mat(sz, DepthType.Cv8U, 1, _handle.AddrOfPinnedObject(), sz.Width);
        }
        public void Free()
        {
            _mat = null;
            _data = null;
            _handle.Free();
        }
    }
   class Program
   {
       static MyMat NormalizeSudokuMat(Mat sudoku)
        {
            MyMat normSudoku = new MyMat(sudoku.Size);
            
            CvInvoke.GaussianBlur(sudoku, normSudoku._mat, new System.Drawing.Size(11, 11), 0);
            CvInvoke.AdaptiveThreshold(normSudoku._mat, normSudoku._mat, 255, AdaptiveThresholdType.MeanC, ThresholdType.BinaryInv, 5, 2);
            byte[,] dat = new byte[3, 3] { { 0, 1, 0, }, { 1, 1, 1, }, { 0, 1, 0 } };
            Mat kernel = new Mat();
            kernel.SetTo(dat);
            CvInvoke.Dilate(normSudoku._mat, normSudoku._mat, kernel, new Point(), 1, BorderType.Default, new MCvScalar());

            int max = -1;
            Point maxPt = new Point();
            Rectangle rect;
            int idx = 0;
            for (int y = 0; y < normSudoku._mat.Size.Height; y++)
            {
                for (int x = 0; x < normSudoku._mat.Size.Width; x++)
                {
                    if (normSudoku._data[idx++] >= 128)
                    {
                        int area = CvInvoke.FloodFill(normSudoku._mat, null, new Point(x, y), new MCvScalar(64, 0, 0), out rect, new MCvScalar(), new MCvScalar());
                        if (area > max)
                        {
                            maxPt.X = x;
                            maxPt.Y = y;
                            max = area;
                        }
                    }
                }
            }
            CvInvoke.FloodFill(normSudoku._mat, null, maxPt, new MCvScalar(255, 255, 255), out rect, new MCvScalar(), new MCvScalar());
            idx = 0;
            for (int y = 0; y < normSudoku._mat.Size.Height; y++)
            {
                for (int x = 0; x < normSudoku._mat.Size.Width; x++)
                {
                    if (normSudoku._data[idx++] == 64 && x != maxPt.X && y != maxPt.Y)
                    {
                        int area = CvInvoke.FloodFill(normSudoku._mat, null, new Point(x, y), new MCvScalar(0, 0, 0), out rect, new MCvScalar(), new MCvScalar());
                    }
                }
                //printf("Current row: %d\n", y);
            }
            CvInvoke.Erode(normSudoku._mat, normSudoku._mat, kernel, new Point(), 1, BorderType.Default, new MCvScalar());

            Emgu.CV.Util.VectorOfPointF vp = new Emgu.CV.Util.VectorOfPointF();
            CvInvoke.HoughLines(normSudoku._mat, vp, 1, Math.PI / 180, 200);
            //CvInvoke.HoughLinesP(normSudoku._mat, vp, 1, Math.PI / 180, 80, normSudoku._mat.Size.Width / 2.0, 50);
            PointF[] lines = vp.ToArray();
            lines = sudoku.mergeRelatedLines(lines, sudoku.Width / (9 * 4.0f), (float)(15 * Math.PI / 180));
            List<List<int>> ret = SudokuMatHelper.classifyLines(lines, (float)(15 * Math.PI / 180));
            SudokuMatHelper.ridRedundantLines(lines, ret);
#if DebugMat
            int cntLines = ret[1].Count;
            for (int i = 0; i < cntLines; i++)
            {
                normSudoku._mat.drawLine(lines[ret[1][i]], new MCvScalar(128, 0, 0));
            }
#endif
            normSudoku.Free();
            PointF[] outlinePoints = sudoku.getIntersectionOutline(lines, ret[0], ret[1]);
            int nPoints = outlinePoints.Length;
            float maxLen = (outlinePoints[nPoints - 1].X - outlinePoints[0].X) * (outlinePoints[nPoints - 1].X - outlinePoints[0].X) +
                        (outlinePoints[nPoints - 1].Y - outlinePoints[0].Y) * (outlinePoints[nPoints - 1].Y - outlinePoints[0].Y);
            for (int i = 0; i < outlinePoints.Length - 1; i++)
            {
                float len = (outlinePoints[i + 1].X - outlinePoints[i].X) * (outlinePoints[i + 1].X - outlinePoints[i].X) +
                            (outlinePoints[i + 1].Y - outlinePoints[i].Y) * (outlinePoints[i + 1].Y - outlinePoints[i].Y);
                if (len > maxLen)
                {
                    maxLen = len;
                }
            }
            maxLen = (float)Math.Sqrt(maxLen);
            PointF[] dstRectPoints = new PointF[]{
                new PointF(0,0),
                new PointF(maxLen,0),
                new PointF(maxLen,maxLen),
                new PointF(0,maxLen),
            };
            Size szRect = new Size((int)maxLen, (int)maxLen);
            normSudoku = new MyMat(szRect);
            CvInvoke.WarpPerspective(sudoku, normSudoku._mat, CvInvoke.GetPerspectiveTransform(outlinePoints, dstRectPoints), szRect);
            return normSudoku;
        }
       static void MakeupForOCR(MyMat normSudoku)
       {
           CvInvoke.GaussianBlur(normSudoku._mat, normSudoku._mat, new System.Drawing.Size(3, 3), 0);
           CvInvoke.AdaptiveThreshold(normSudoku._mat, normSudoku._mat, 255, AdaptiveThresholdType.GaussianC, ThresholdType.BinaryInv, 5, 2);
           byte[,] dat = new byte[3, 3] { { 0, 1, 0, }, { 1, 1, 1, }, { 0, 1, 0 } };
           Mat kernel = new Mat();
           kernel.SetTo(dat);
           CvInvoke.Dilate(normSudoku._mat, normSudoku._mat, kernel, new Point(), 1, BorderType.Default, new MCvScalar());

            int max = normSudoku._mat.Size.Width / 9;
           max *= max;
           Point maxPt = new Point();
           Rectangle rect;
           int idx = 0;
           for (int y = 0; y < normSudoku._mat.Size.Height; y++)
           {
               for (int x = 0; x < normSudoku._mat.Size.Width; x++)
               {
                   if (normSudoku._data[idx++] >= 255)
                   {
                       int area = CvInvoke.FloodFill(normSudoku._mat, null, new Point(x, y), new MCvScalar(254, 0, 0), out rect, new MCvScalar(), new MCvScalar());
                       if (area > max)
                       {
                           CvInvoke.FloodFill(normSudoku._mat, null, new Point(x, y), new MCvScalar(0, 0, 0), out rect, new MCvScalar(), new MCvScalar());
                       }
                   }
               }
           }

           CvInvoke.Erode(normSudoku._mat, normSudoku._mat, kernel, new Point(), 1, BorderType.Default, new MCvScalar());
       }

        static void Main(string[] args)
        {
            string strSudokuPic = "../tstData/sudoku-original.jpg";
            //strSudokuPic = "../tstData/111.jpg";
            strSudokuPic = "../tstData/222.jpg";
            Mat sudoku = new Mat(strSudokuPic, LoadImageType.Grayscale);
            MyMat normSudoku = NormalizeSudokuMat(sudoku);
            //Mat[] chs = img.Split();
            //CvInvoke.Add(chs[0], chs[1], chs[1]);
            //CvInvoke.Subtract(chs[2], chs[1], chs[2]);            

            MakeupForOCR(normSudoku);
            
            string strNums = "0123456789";
            Emgu.CV.OCR.Tesseract ocr = new Emgu.CV.OCR.Tesseract("", "eng", Emgu.CV.OCR.OcrEngineMode.TesseractOnly, strNums);
            ocr.Recognize(normSudoku._mat);
            float digitSize = normSudoku._mat.Width / 9.0f;
            Emgu.CV.OCR.Tesseract.Character[] characters = ocr.GetCharacters();
            foreach (Emgu.CV.OCR.Tesseract.Character c in characters)
            {
                if (strNums.Contains(c.Text) && c.Region.Width < digitSize && c.Region.Height < digitSize)
                {
                    float top = c.Region.Top / digitSize;
                    float btm = c.Region.Bottom / digitSize;
                    int num;
                    if(top - (int)top < 0.3f && btm - (int)btm > 0.7f && int.TryParse(c.Text, out num))
                    {
                        float left = c.Region.Left / digitSize;
                        SudokuSolver.setSudoku((int)top, (int)left, num);
#if DebugMat
                        CvInvoke.Rectangle(normSudoku._mat, c.Region, new MCvScalar(255, 0, 0));
#endif
                    }
                }
            }
            if(SudokuSolver.solveSudoku())
            {
                for(int i = 0; i < 9; i++)
                {
                    for(int j = 0; j < 9; j++)
                    {
                        CvInvoke.PutText(normSudoku._mat, SudokuSolver.getSudoku(i,j).ToString(), new Point((int)((j+0.5)*digitSize), (int)((i+0.5)*digitSize)), FontFace.HersheyPlain, 1, new MCvScalar(255,0,0));
                    }
                }
            }
            CvInvoke.Imshow("Ori", sudoku);
            CvInvoke.Imshow("out", normSudoku._mat);
            CvInvoke.WaitKey(0);
            CvInvoke.DestroyAllWindows();
            return;
        }
/*
 *     static void Main(string[] args)
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
 */
   }
}