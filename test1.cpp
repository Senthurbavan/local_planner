#include <vector>
#include <iostream>

using namespace std;

static const unsigned char ROW = 3;
static const unsigned char COL = 5;

struct GridCell
{
    int target_dist;
    bool target_mark;
};

bool checkCell(GridCell& cell)
{
    cell.target_dist = 123;
    cell.target_mark = true;
}


int main()
{
    vector<vector<GridCell>> grid1(ROW, vector<GridCell>(COL));


    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            cout << "(" << grid1[i][j].target_dist;
            cout << ", " << grid1[i][j].target_mark << ") ";
        }
        cout << endl;
        
    }

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            checkCell(grid1[i][j]);
        }
        
    }

    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            cout << "(" << grid1[i][j].target_dist;
            cout << ", " << grid1[i][j].target_mark << ") ";
        }
        cout << endl;
        
    }
    
    
    return 0;
}