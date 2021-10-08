#include <vector>
#include <queue>
#include <iostream>

using namespace std;

static const unsigned char ROW = 3; //y
static const unsigned char COL = 5; //x

struct GridCell
{
    int cx, cy;
    int target_dist;
    bool target_mark;
};

vector<vector<GridCell>> pathmap(ROW, vector<GridCell>(COL));
vector<vector<GridCell>> goalmap(ROW, vector<GridCell>(COL));
vector<vector<int>> costmap(ROW, vector<int>(COL, 0));   


void initializeGrid(vector<vector<GridCell>>& grid)
{
    for (int i = 0; i < ROW; i++)
    {
        for (int j = 0; j < COL; j++)
        {
            grid[i][j].cx = j;
            grid[i][j].cy = i;
            grid[i][j].target_dist = __INT_MAX__;
            grid[i][j].target_mark = false;
        }
        
    }
    
}


void computeTargetDistance(queue<GridCell*>& dist_queue, vector<vector<int>>& costmap)
{

}



int main()
{   
    cout << pathmap.size() << endl;
    cout << costmap.size() << endl;
    return 0;
}