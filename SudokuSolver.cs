using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OpticalSudokuSolver
{
    public static class SudokuSolver
    {
        const int UndefinedNum = 0;
        static int[,] _datMat = new int[9, 9];
        static bool isValidSudoku(int[,] board, int r, int c)
        {
            int row, col;
            // Same value in the same column?
            for (row = 0; row < 9; ++row)
            {
                if ((r != row) && (board[row,c] == board[r,c]))
                {
                    return false;
                }
            }
            // Same value in the same row?
            for (col = 0; col < 9; ++col)
            {
                if ((c != col) && (board[r,col] == board[r,c]))
                {
                    return false;
                }
            }
            // Same value in the 3 * 3 block it belong to?
            for (row = (r / 3) * 3; row < (r / 3 + 1) * 3; ++row)
            {
                for (col = (c / 3) * 3; col < (c / 3 + 1) * 3; ++col)
                {
                    if ((r != row) && (c != col) && (board[row,col] == board[r,c]))
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        static bool internalSolveSudoku(int[,] board)
        {
            for (int row = 0; row < 9; ++row)
            {
                for (int col = 0; col < 9; ++col)
                {
                    if (UndefinedNum == board[row,col])
                    {
                        for (int i = 1; i <= 9; ++i)
                        {
                            board[row,col] = i;

                            if (isValidSudoku(board, row, col))
                            {
                                if (internalSolveSudoku(board))
                                {
                                    return true;
                                }
                            }

                            board[row,col] = UndefinedNum;
                        }

                        return false;
                    }
                }
            }

            return true;
        }

        public static void setSudoku(int row, int col, int num)
        {
            _datMat[row, col] = num;
        }
        public static int getSudoku(int row, int col)
        {
            return _datMat[row, col];
        }
        public static bool solveSudoku()
        {
            return internalSolveSudoku(_datMat);
        }
    }
}
