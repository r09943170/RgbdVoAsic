#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <cstdlib>
using namespace std;

int main(){

string FN[27];
// FN[ 0] = "debug_check_A0_mul_A0";
// FN[ 1] = "debug_check_A0_mul_A1";
// FN[ 2] = "debug_check_A0_mul_A2";
// FN[ 3] = "debug_check_A0_mul_A3";
// FN[ 4] = "debug_check_A0_mul_A4";
// FN[ 5] = "debug_check_A0_mul_A5";
// FN[ 6] = "debug_check_A1_mul_A1";
// FN[ 7] = "debug_check_A1_mul_A2";
// FN[ 8] = "debug_check_A1_mul_A3";
// FN[ 9] = "debug_check_A1_mul_A4";
// FN[10] = "debug_check_A1_mul_A5";
// FN[11] = "debug_check_A2_mul_A2";
// FN[12] = "debug_check_A2_mul_A3";
// FN[13] = "debug_check_A2_mul_A4";
// FN[14] = "debug_check_A2_mul_A5";
// FN[15] = "debug_check_A3_mul_A3";
// FN[16] = "debug_check_A3_mul_A4";
// FN[17] = "debug_check_A3_mul_A5";
// FN[18] = "debug_check_A4_mul_A4";
// FN[19] = "debug_check_A4_mul_A5";
// FN[20] = "debug_check_A5_mul_A5";
// FN[21] = "debug_check_A0_mul_diff_div_w";
// FN[22] = "debug_check_A1_mul_diff_div_w";
// FN[23] = "debug_check_A2_mul_diff_div_w";
// FN[24] = "debug_check_A3_mul_diff_div_w";
// FN[25] = "debug_check_A4_mul_diff_div_w";
// FN[26] = "debug_check_A5_mul_diff_div_w";

FN[ 0] = "debug_M_00";
FN[ 1] = "debug_M_10";
FN[ 2] = "debug_M_20";
FN[ 3] = "debug_M_30";
FN[ 4] = "debug_M_40";
FN[ 5] = "debug_M_50";
FN[ 6] = "debug_M_11";
FN[ 7] = "debug_M_21";
FN[ 8] = "debug_M_31";
FN[ 9] = "debug_M_41";
FN[10] = "debug_M_51";
FN[11] = "debug_M_22";
FN[12] = "debug_M_32";
FN[13] = "debug_M_42";
FN[14] = "debug_M_52";
FN[15] = "debug_M_33";
FN[16] = "debug_M_43";
FN[17] = "debug_M_53";
FN[18] = "debug_M_44";
FN[19] = "debug_M_54";
FN[20] = "debug_M_55";
FN[21] = "debug_V_0";
FN[22] = "debug_V_1";
FN[23] = "debug_V_2";
FN[24] = "debug_V_3";
FN[25] = "debug_V_4";
FN[26] = "debug_V_5";

for (int i = 0; i < 27;i++){
    // string file_name = "debug_check_A0_mul_A0";
    string file_name = FN[i];
    string file_HW = "./" + file_name + "_HW.txt";
    string file_SW = "./" + file_name + "_SW.txt";
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
}