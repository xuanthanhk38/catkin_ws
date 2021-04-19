#ifndef _ACT_FILE_H
#define _ACT_FILE_H

#include <fstream>
#include <iostream>
#include <vector>

using namespace std;
char *filePath = "/home/tma/catkin_ws/src/odom_sub/file/a.csv";
char *filePath2 = "/home/tma/catkin_ws/src/odom_sub/file/b.csv";
char *filePath3 = "/home/tma/catkin_ws/src/odom_sub/file/c.csv";
//extern float data_from_file[300];
extern vector<float> data_from_file; 


void WriteToFile(float pos_x, float pos_y, float orien_z, float orien_w)
{

  ofstream file;
  //float data;
  file.open(filePath, fstream::app); //use binary mode
  if (file.is_open())
  {
    ROS_INFO("File is open");
  }
  else
  {
    ROS_INFO("Can not open this file");
  }

  file<<pos_x<<'\n';
  file<<pos_y<<'\n';
  file<<orien_z<<'\n';
  file<<orien_w<<'\n';
  file.close();
}

void SaveToArray(int size, char *destinationfile)
{
  //int size1 = 0;
  //size1 = size;
  float my_arr[size];
  int cnt = 0;
  //int line_num = 0;
  float line;
  float read_data;

  ifstream file1(destinationfile);//2
  if(file1.is_open())
  {
    //ROS_INFO("Read file");

    while(cnt < size && file1 >> read_data)
    {
      my_arr[cnt++] = read_data;
      //ROS_INFO("cnt = %d", cnt);
    }

    for (int i = 0; i < cnt; i++)
    {
      //data_from_file[i] = my_arr[i];
      data_from_file.push_back(my_arr[i]);
      //ROS_INFO("Arr[%d] = %f", i, my_arr[i]);
    }

    file1.close();
  }
}

void copy_file(char *sourcefile, char *destinationfile)
{
  ifstream fs;
  ofstream ft;
  string str;
  fs.open(sourcefile);

  if (!fs)
  {
    cout << "Error in Opening Source File...!!!";
    exit(1);
  }


  ft.open(destinationfile);

  if (!ft)
  {
    cout << "Error in Opening Destination File...!!!";
    fs.close();
    exit(2);
  }

  if (fs && ft)
  {
    while (getline(fs, str))
    {
      ft << str << "\n";
    }

    cout << "\n\n Source File Date Successfully Copied to Destination File...!!!";

  }
  else
  {
    cout << "File Cannot Open...!!!";
  }

  cout << "\n\n Open Destination File and Check!!!\n";

  fs.close();
  ft.close();
}

int count_line(char *sourcefile)
{
  string line;
  int line_num = 0;

  ifstream file(sourcefile); //use binary mode
  if(file.is_open())
  {
    while(getline(file, line))
    {
      line_num++;
    }
    ROS_INFO("line_num: %d", line_num);
  }
  return line_num;
}
#endif //_ACT_FILE_H