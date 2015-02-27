#include <ros/ros.h>
#include <fstream>

int main()
{
    double targ_x, targ_y, targ_z;
    int lineCount = 0;
    std::string read_x, read_y, read_z, line;
    std::ifstream input("output.txt");

    while (getline(input,line))
        lineCount++;
    std::cout << "Found "<< lineCount << " recorded marker points.\n";
    input.close();
    input.open("output.txt");

    for (int i=0;i<lineCount;i++)
    {
        getline(input,read_x,',');
        getline(input,read_y,',');
        getline(input,read_z);
        targ_x = atof(read_x.c_str());
        targ_y = atof(read_y.c_str());
        targ_z = atof(read_z.c_str());

        deictic_event.target_pos.set(targ_x,targ_y,targ_z);
        temp.push_back(deictic_event);
    }
    input.close();
    return 0;
}
