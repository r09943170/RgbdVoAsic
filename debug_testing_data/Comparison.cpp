#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <cstdlib>
using namespace std;

int main(){

string FN[69];
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

// FN[ 0] = "debug_Ax_0";
// FN[ 1] = "debug_Ax_1";
// FN[ 2] = "debug_Ax_2";
// FN[ 3] = "debug_Ax_3";
// FN[ 4] = "debug_Ax_4";
// FN[ 5] = "debug_Ax_5";
// FN[ 6] = "debug_Ay_0";
// FN[ 7] = "debug_Ay_1";
// FN[ 8] = "debug_Ay_2";
// FN[ 9] = "debug_Ay_3";
// FN[10] = "debug_Ay_4";
// FN[11] = "debug_Ay_5";
// FN[12] = "debug_diff_x";
// FN[13] = "debug_diff_y";
// FN[14] = "debug_M_00";
// FN[15] = "debug_M_10";
// FN[16] = "debug_M_20";
// FN[17] = "debug_M_30";
// FN[18] = "debug_M_40";
// FN[19] = "debug_M_50";
// FN[20] = "debug_M_11";
// FN[21] = "debug_M_21";
// FN[22] = "debug_M_31";
// FN[23] = "debug_M_41";
// FN[24] = "debug_M_51";
// FN[25] = "debug_M_22";
// FN[26] = "debug_M_32";
// FN[27] = "debug_M_42";
// FN[28] = "debug_M_52";
// FN[29] = "debug_M_33";
// FN[30] = "debug_M_43";
// FN[31] = "debug_M_53";
// FN[32] = "debug_M_44";
// FN[33] = "debug_M_54";
// FN[34] = "debug_M_55";
// FN[35] = "debug_V_0";
// FN[36] = "debug_V_1";
// FN[37] = "debug_V_2";
// FN[38] = "debug_V_3";
// FN[39] = "debug_V_4";
// FN[40] = "debug_V_5";
// FN[41] = "debug_LDLT_Mat_00";
// FN[42] = "debug_LDLT_Mat_10";
// FN[43] = "debug_LDLT_Mat_20";
// FN[44] = "debug_LDLT_Mat_30";
// FN[45] = "debug_LDLT_Mat_40";
// FN[46] = "debug_LDLT_Mat_50";
// FN[47] = "debug_LDLT_Mat_11";
// FN[48] = "debug_LDLT_Mat_21";
// FN[49] = "debug_LDLT_Mat_31";
// FN[50] = "debug_LDLT_Mat_41";
// FN[51] = "debug_LDLT_Mat_51";
// FN[52] = "debug_LDLT_Mat_22";
// FN[53] = "debug_LDLT_Mat_32";
// FN[54] = "debug_LDLT_Mat_42";
// FN[55] = "debug_LDLT_Mat_52";
// FN[56] = "debug_LDLT_Mat_33";
// FN[57] = "debug_LDLT_Mat_43";
// FN[58] = "debug_LDLT_Mat_53";
// FN[59] = "debug_LDLT_Mat_44";
// FN[60] = "debug_LDLT_Mat_54";
// FN[61] = "debug_LDLT_Mat_55";
// FN[62] = "debug_X0";
// FN[63] = "debug_X1";
// FN[64] = "debug_X2";
// FN[65] = "debug_X3";
// FN[66] = "debug_X4";
// FN[67] = "debug_X5";
// FN[68] = "debug_rodrigues_pose";

// FN[ 0] = "debug_Corr_u0";
// FN[ 1] = "debug_Corr_v0";
// FN[ 2] = "debug_Corr_d0";
// FN[ 3] = "debug_Corr_u1";
// FN[ 4] = "debug_Corr_v1";
// FN[ 5] = "debug_Corr_d1";
// FN[ 6] = "debug_Corr_n1_x";
// FN[ 7] = "debug_Corr_n1_y";
// FN[ 8] = "debug_Corr_n1_z";
// FN[ 9] = "debug_Corr_dI_dx";
// FN[10] = "debug_Corr_dI_dy";
// FN[11] = "debug_Corr_data0";
// FN[12] = "debug_Corr_data1";
FN[13] = "debug_ICP_A0";
FN[14] = "debug_ICP_A1";
FN[15] = "debug_ICP_A2";
FN[16] = "debug_ICP_A3";
FN[17] = "debug_ICP_A4";
FN[18] = "debug_ICP_A5";
FN[19] = "debug_ICP_diff_div_w";

for (int i = 13; i <= 19; i++){
    // string file_name = "debug_check_A0_mul_A0";
    string file_name = FN[i];
    // string file_HW = "./" + file_name + "_HW.txt";
    // string file_SW = "./" + file_name + "_SW.txt";
    string file_HW = "./HW_" + file_name + ".txt";
    string file_SW = "./SW_" + file_name + ".txt";
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