#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <cstdlib>
using namespace std;

int main(){
    string file_name = "debug_check_diff_s_sum";
    // string file_HW = file_name + "_HW.txt";
    // string file_SW = file_name + "_SW.txt";
    string file_HW = "./debug_testing_data/v1.txt";
    string file_SW = "./debug_testing_data/feature_v1.txt";
    //HW : 
    ifstream fi_1(file_HW);
    bool check_input_path_1 = fi_1.good();
    if (check_input_path_1 == 0) {
        cout << file_HW << " doesn't exist." << endl;
        exit(1);
    }
    //SW :
    ifstream fi_2(file_SW); 
    bool check_input_path_2 = fi_2.good();
    if (check_input_path_2 == 0) {
        cout << file_SW << " doesn't exist." << endl;
        exit(1);
    }
    //Result report : 
    ofstream fo("debug_Comparison_Result.txt");
    string fin_1, fin_2;
    int Count_diff = 0;
    int Count_line = 0;
    int first_diff = 0;
    while (getline(fi_1,fin_1)){
        Count_line++;
        getline(fi_2,fin_2);
        if (atoi(fin_1.c_str()) != atoi(fin_2.c_str())) {
        // if (abs(atoi(fin_1.c_str()) - atoi(fin_2.c_str()))/abs(atoi(fin_1.c_str())) > 0.0001) {
            fo << Count_line << endl;
            Count_diff++;
            if (Count_diff == 1) first_diff = Count_line;
        }
    }
    cout << file_name + ": " << endl;
    cout << "   Count_data = " << Count_line << endl;
    cout << "   Count_diff = " << Count_diff << endl;
    if (Count_diff > 0) cout << "   First_diff = " << first_diff << endl;
    else cout << "   First_diff = none" << endl;
}