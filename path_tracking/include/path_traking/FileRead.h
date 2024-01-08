//
// Created by hhlt on 2020/1/6.
//from file of ".csv" to array by C++
//
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

#ifndef UNTITLED1_FILEREAD_H
#define UNTITLED1_FILEREAD_H

using namespace std;
class FileRead{

private:
    vector<vector<string> > strArray;
    string str="";

public:

    FileRead(string temp) :str(temp){

    }

  bool read_file(){
        ifstream inFile(str, ios::in);
        string lineStr;
        char flag=';';
        if(inFile.fail()){
            cout <<"Read files fail....."<<endl;
            return false;
        }

        while (getline(inFile, lineStr)) {
            stringstream ss(lineStr);
            string str;
            vector<string> lineArray;
            // cut apart by flag
            while (getline(ss, str, flag))
                lineArray.push_back(str);
            strArray.push_back(lineArray);
        }

      return true;
    }



    void show_str_array(int row,int col){

        if(!read_file()){
            cout<<"No array!"<<endl;
            cout<<"print array fail!"<<endl;
            return;
        }
            vector<string>::iterator it;
            vector<vector<string> >::iterator iter;
            vector<string> vec_tmp;

            for(iter=strArray.begin()+row; iter != strArray.end(); iter++)
            {
                vec_tmp = *iter;
                for(it = vec_tmp.begin()+col; it != vec_tmp.end(); it++)
                    cout << *it <<"\t";
                cout << endl;
            }
        cout<<endl<<"print array success !"<<endl;
    }





    //another way to show strArray
    void show_str_array_2(int row,int col){

        //from row and col to show strArray
        for (int i = row; i <strArray.size() ; ++i) {
            for (int j = col; j <strArray[0].size() ; ++j) {
                cout<<strArray[i][j]    <<"\t"    ;    }

            cout<<endl;
        }

    }


};


#endif //UNTITLED1_FILEREAD_H
