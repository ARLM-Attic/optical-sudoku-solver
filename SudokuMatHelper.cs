﻿using System;
using System.Collections.Generic;
using System.Drawing;
using System.Runtime.InteropServices;
using Emgu.CV;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;

namespace OpticalSudokuSolver
{
    public static class SudokuMatHelper
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
        public static void getBegEnd(this Mat img, PointF line, out Point beg, out Point end)
        {
#if true
            double tan = Math.Tan(line.Y);

            float m = Math.Abs(line.Y / ((float)(Math.PI)));
            m -= (float)Math.Floor(m);
            if (m > 0.25f && m < 0.75f)
            {
                float c = (float)(line.X / Math.Sin(line.Y));
                beg = new Point(0, (int)c);
                end = new Point(img.Size.Width, (int)(c - img.Size.Width / tan));
            }
            else
            {
                float c = (float)(line.X / Math.Cos(line.Y));
                beg = new Point((int)c, 0);
                end = new Point((int)(c - img.Size.Height * tan), img.Size.Height);
            }
#else
            if (line.Y != 0)
            {
                float m = (float)(-1.0 / Math.Tan(line.Y));
                float c = (float)(line.X / Math.Sin(line.Y));
                beg = new Point(0, (int)c);
                end = new Point(img.Size.Width, (int)(m * img.Size.Width + c));
            }
            else
            {
                beg = new Point((int)line.X, 0);
                end = new Point((int)line.X, img.Size.Height);
            }
#endif
        }
        public static void drawLine(this Mat img, PointF line, MCvScalar rgb)
        {
            Point beg, end;
            img.getBegEnd(line, out beg, out end);
            CvInvoke.Line(img, beg, end, rgb);
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
                if (idx < 0)
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
                    if (theta < 0)
                    {
                        theta += (float)(2 * Math.PI);
                    }
                    else if (theta > (float)(2 * Math.PI))
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
        public static List<List<int>> classifyLines(PointF[] lines, float maxDeltaTheta)
        {
            int n = lines.Length;
            List<int> idxLst = new List<int>();
            for (int i = 0; i < n; i++)
            {
                idxLst.Add(i);
            }
            List<float> thetas = new List<float>();
            List<List<int>> ret = new List<List<int>>();
            //Classify lines based by thetas
            while (idxLst.Count > 0)
            {
                List<int> curRet = new List<int>();
                int idx = idxLst[idxLst.Count - 1];
                float theta = lines[idx].Y;
                idxLst.RemoveAt(idxLst.Count - 1);
                curRet.Add(idx);
                thetas.Add(theta);

                for (int i = idxLst.Count - 1; i >= 0; i--)
                {
                    idx = idxLst[i];
                    float theta1 = lines[idx].Y;
                    float dTheta = Math.Abs(theta - theta1);
                    if (dTheta > (float)Math.PI)
                    {
                        dTheta = (float)(2 * Math.PI) - dTheta;
                    }
                    if (dTheta < maxDeltaTheta)
                    {
                        idxLst.RemoveAt(i);
                        curRet.Add(idx);
                    }
                }
                ret.Add(curRet);
            }
            // Find the first 2 otho lines group
            for (int i = 0; i < thetas.Count; i++)
            {
                int idx = thetas.FindIndex(th =>
                {
                    float dTheta = Math.Abs(th - thetas[i]);
                    return Math.Abs(dTheta - (float)(0.5 * Math.PI)) < maxDeltaTheta || Math.Abs(dTheta - (float)(1.5 * Math.PI)) < maxDeltaTheta;
                });
                if (idx >= 0)
                {
                    List<int> ret0 = ret[i];
                    List<int> ret1 = ret[idx];
                    ret.Clear();
                    float m = Math.Abs(thetas[i] / ((float)(Math.PI)));
                    m -= (float)Math.Floor(m);
                    if (m > 0.25f && m < 0.75f)
                    {
                        ret.Add(ret1);
                        ret.Add(ret0);
                    }
                    else
                    {
                        ret.Add(ret0);
                        ret.Add(ret1);
                    }
                    break;
                }
            }
            return ret;
        }
        public static bool ridRedundantLines(PointF[] lines, List<List<int>> grps)
        {
            if (grps[0].Count == grps[1].Count && (grps[0].Count == 4 || grps[0].Count == 10))
                return true;

            // Asssume the first line is sudoku rect line
            for(int i = 0; i < 2; i++)
            {
                List<int> grp = grps[i];
                grp.Sort((it0, it1) => (int)(lines[it0].X - lines[it1].X));

                bool fin = false;
                for (int si = 0; si < grp.Count - 3 && !fin; si++)
                {
                    for(int ei = grp.Count-1; ei >= si + 3 && !fin; ei--)
                    {
                        float gridLen = (lines[grp[ei]].X - lines[grp[si]].X)/3;
                        float maxDeltaP = gridLen / (3 * 2.5f);
                        for (int j0 = si + 1; j0 < ei && !fin; j0++)
                        {
                            float tmpLen = lines[grp[j0]].X - lines[grp[si]].X;
                            if (Math.Abs(tmpLen - gridLen) < maxDeltaP)
                            {
                                for (int j1 = j0 + 1; j1 < ei && !fin; j1++)
                                {
                                    tmpLen = lines[grp[j1]].X - lines[grp[j0]].X;
                                    if (Math.Abs(tmpLen - gridLen) < maxDeltaP)
                                    {
                                        // Ok ,first remove tails, then heads
                                        int idx0 = grp[si];
                                        int idx1 = grp[j0];
                                        int idx2 = grp[j1];
                                        int idx3 = grp[ei];
                                        grp.Clear();
                                        grp.Add(idx0);
                                        grp.Add(idx1);
                                        grp.Add(idx2);
                                        grp.Add(idx3);
                                        fin = true;
                                    }
                                }
                            }
                        }                        
                    }
                }
                if (!fin)
                    return false;
            }
            return true;
        }
        // Given 2 line segments, find their intersection point
        // rerurns [Px,Py] point in 'res' or FALSE if parallel. Uses vector cross product technique.
        public static bool findLinesIntersectionPoint(Point beg0, Point end0, Point beg1, Point end1, ref PointF intersectionPoint)
        {
            Point dp = new Point(end0.X - beg0.X, end0.Y - beg0.Y);
            Point dq = new Point(end1.X - beg1.X, end1.Y - beg1.Y);
            Point qmp = new Point(beg1.X - beg0.X, beg1.Y - beg0.Y);
            int dpdq_cross = dp.X * dq.Y - dp.Y * dq.X;
            if (dpdq_cross == 0)
                return false;

            int qpdq_cross = qmp.X * dq.Y - qmp.Y * dq.X;
            float a = (qpdq_cross * 1.0f / dpdq_cross);
            intersectionPoint.X = beg0.X + a * dp.X;
            intersectionPoint.Y = beg0.Y + a * dp.Y;
            return true;
        }
        public static PointF[] getAllIntersectionPoints(this Mat img, PointF[] lines)
        {
            int cnt = lines.Length;
            Point[] pBegs = new Point[cnt];
            Point[] pEnds = new Point[cnt];
            for (int i = 0; i < cnt; i++)
            {
                if (lines[i].Y != 0)
                {
                    float m = (float)(-1.0 / Math.Tan(lines[i].Y));
                    float c = (float)(lines[i].X / Math.Sin(lines[i].Y));
                    pBegs[i] = new Point(0, (int)c);
                    pEnds[i] = new Point(img.Size.Width, (int)(m * img.Size.Width + c));
                }
                else
                {
                    pBegs[i] = new Point((int)lines[i].X, 0);
                    pEnds[i] = new Point((int)lines[i].X, img.Size.Height);
                }
            }
            List<PointF> lstIntersectionPoints = new List<PointF>();
            for (int i = 0; i < cnt; i++)
            {
                for (int j = i + 1; j < cnt; j++)
                {
                    PointF ret = new PointF();
                    if (findLinesIntersectionPoint(pBegs[i], pEnds[i], pBegs[j], pEnds[j], ref ret) && ret.X > 0 && ret.Y > 0)
                    {
                        lstIntersectionPoints.Add(ret);
                    }
                }
            }
            return lstIntersectionPoints.ToArray();
        }
        public static PointF[] getIntersectionOutline(this Mat img, PointF[] lines, List<int> grp0, List<int> grp1)
        {
            int idxMax0 = grp0[0];
            int idxMin0 = grp0[0];
            for (int i = 1; i < grp0.Count; i++)
            {
                if (lines[grp0[i]].X > lines[idxMax0].X)
                {
                    idxMax0 = grp0[i];
                }
                if (lines[grp0[i]].X < lines[idxMin0].X)
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

            Point begMin0, begMax0, endMin0, endMax0;
            Point begMin1, begMax1, endMin1, endMax1;
            img.getBegEnd(lines[idxMin0], out begMin0, out endMin0);
            img.getBegEnd(lines[idxMax0], out begMax0, out endMax0);
            img.getBegEnd(lines[idxMin1], out begMin1, out endMin1);
            img.getBegEnd(lines[idxMax1], out begMax1, out endMax1);

            PointF[] ret = new PointF[4];
            findLinesIntersectionPoint(begMin0, endMin0, begMin1, endMin1, ref ret[0]);
            findLinesIntersectionPoint(begMax0, endMax0, begMin1, endMin1, ref ret[1]);
            findLinesIntersectionPoint(begMax0, endMax0, begMax1, endMax1, ref ret[2]);
            findLinesIntersectionPoint(begMin0, endMin0, begMax1, endMax1, ref ret[3]);
            return ret;
        }
    }
}
