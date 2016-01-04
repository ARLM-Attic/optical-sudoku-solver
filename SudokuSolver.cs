using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace OpticalSudokuSolver
{
    public static class SudokuSolver
    {
        const int UndefinedNum = -1;
        static int[,] _datMat = new int[9, 9];
        static bool isValidSudoku(int[,] board, int x, int y)
        {
            int row, col;
            // Same value in the same column?
            for (row = 0; row < 9; ++row)
            {
                if ((x != row) && (board[row,y] == board[x,y]))
                {
                    return false;
                }
            }
            // Same value in the same row?
            for (col = 0; col < 9; ++col)
            {
                if ((y != col) && (board[x,col] == board[x,y]))
                {
                    return false;
                }
            }
            // Same value in the 3 * 3 block it belong to?
            for (row = (x / 3) * 3; row < (x / 3 + 1) * 3; ++row)
            {
                for (col = (y / 3) * 3; col < (y / 3 + 1) * 3; ++col)
                {
                    if ((x != row) && (y != col) && (board[row,col] == board[x,y]))
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

        public static void setSudoku(int x, int y, int num)
        {
            _datMat[x, y] = num;
        }
        public static void solveSudoku()
        {
            internalSolveSudoku(_datMat);
        }
    }
}
