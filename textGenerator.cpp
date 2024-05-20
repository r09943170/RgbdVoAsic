#include <iostream>
#include <fstream>
#include <stdio.h>
#include <string>
#include <cstdlib>
using namespace std;

int main()
{
    // //dstFrame_lb_store
    // for (int i = 0; i <= 62; i++)
    // {
    //     cout << "                6'd" << i << " : begin" << endl;
    //     cout << "                    if (u1_r_d1[0] == 0) begin" << endl;
    //     cout << "                        for(int i = 0; i <= " << i - 1 << "; i = i + 1)begin" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_WENA_r[i] <= 1;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_DA_r[i]   <= 0;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_AA_r[i]   <= 0;" << endl;
    //     cout << "                        end" << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_WENA_r[" << i << "] <= 0;    " << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_DA_r[" << i << "]   <= o_dstFrame_lb_sram_DA_w;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_AA_r[" << i << "]   <= dstFrame_lb_store_addr;" << endl;
    //     cout << "                        for(int i = " << i + 1 << "; i <= 62; i = i + 1)begin" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_WENA_r[i] <= 1;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_DA_r[i]   <= 0;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_AA_r[i]   <= 0;" << endl;
    //     cout << "                        end" << endl;
    //     cout << "                        for(int i = 0; i <= 62; i = i + 1)begin" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_WENA_r[i] <= 1;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_DA_r[i]   <= 0;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_AA_r[i]   <= 0;" << endl;
    //     cout << "                        end" << endl;
    //     cout << "                    end" << endl;
    //     cout << "                    else begin" << endl;
    //     cout << "                        for(int i = 0; i <= " << i - 1 << "; i = i + 1)begin" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_WENA_r[i] <= 1;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_DA_r[i]   <= 0;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_AA_r[i]   <= 0;" << endl;
    //     cout << "                        end" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_WENA_r[" << i << "] <= 0;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_DA_r[" << i << "]   <= o_dstFrame_lb_sram_DA_w;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_AA_r[" << i << "]   <= dstFrame_lb_store_addr;" << endl;
    //     cout << "                        for(int i = " << i + 1 << "; i <= 62; i = i + 1)begin" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_WENA_r[i] <= 1;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_DA_r[i]   <= 0;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_odd_AA_r[i]   <= 0;" << endl;
    //     cout << "                        end" << endl;
    //     cout << "                        for(int i = 0; i <= 62; i = i + 1)begin" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_WENA_r[i] <= 1;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_DA_r[i]   <= 0;" << endl;
    //     cout << "                            o_dstFrame_lb_sram_even_AA_r[i]   <= 0;" << endl;
    //     cout << "                        end" << endl;
    //     cout << "                    end" << endl;
    //     cout << "                end" << endl;
    // }

    // //dstFrame_lb_load_A
    // cout << "                8'd" << 0 << " : begin" << endl;
    // cout << "                    if (proj_u1_r[0] == 0 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 0 << "] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 0 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 1 << "] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = " << 2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = " << 1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 1 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 0 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 0 << "] <= dstFrame_lb_load_addr + 1;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 1 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = " << 1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = " << 2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 0 && (proj_u1_r >= r_hsize-1 || proj_v1_r >= r_vsize-1)) begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 0 << "] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = " << 1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 0 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = " << 1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                end" << endl;
    // cout << "                8'd" << 1 << " : begin" << endl;
    // cout << "                    if (proj_u1_r[0] == 0 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 0 << "] <= idle_Addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 0 << "]  <= idle_Addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 1 << "] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 1 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 2 << "] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = " << 3 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = " << 2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 1 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 0 << "] <= idle_Addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 0 << "]  <= idle_Addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 1 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 1 << "] <= dstFrame_lb_load_addr + 1;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 2 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = " << 2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = " << 3 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 0 && (proj_u1_r >= r_hsize-1 || proj_v1_r >= r_vsize-1)) begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 0 << "] <= idle_Addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[" << 1 << "] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = " << 2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else begin" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 0 << "]  <= idle_Addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << 1 << "]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = " << 2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                end" << endl;
    // for (int i = 2; i <= 59; i++)
    // {
    //     cout << "                8'd" << i << " : begin" << endl;
    //     cout << "                    if (proj_u1_r[0] == 0 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    //     cout << "                        for(int i = 0; i <= " << i-1 << "; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    //     cout << "                        for(int i = 0; i <= " << i-1 << "; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_AB_r[" << i << "] <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << i << "]  <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_AB_r[" << i+1 << "] <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        for(int i = " << i+2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    //     cout << "                        for(int i = " << i+1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                    end" << endl;
    //     cout << "                    else if (proj_u1_r[0] == 1 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    //     cout << "                        for(int i = 0; i <= " << i-1 << "; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    //     cout << "                        for(int i = 0; i <= " << i-1 << "; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << i << "]  <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_AB_r[" << i << "] <= dstFrame_lb_load_addr + 1;" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << i+1 << "]  <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        for(int i = " << i+1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    //     cout << "                        for(int i = " << i+2 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                    end" << endl;
    //     cout << "                    else if (proj_u1_r[0] == 0 && (proj_u1_r >= r_hsize-1 || proj_v1_r >= r_vsize-1)) begin" << endl;
    //     cout << "                        for(int i = 0; i <= " << i-1 << "; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                        o_dstFrame_lb_sram_even_AB_r[" << i << "]  <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        for(int i = " << i+1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    //     cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                    end" << endl;
    //     cout << "                    else begin" << endl;
    //     cout << "                        for(int i = 0; i <= " << i-1 << "; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                        o_dstFrame_lb_sram_odd_AB_r[" << i << "]  <= dstFrame_lb_load_addr;" << endl;
    //     cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    //     cout << "                        for(int i = " << i+1 << "; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    //     cout << "                    end" << endl;
    //     cout << "                end" << endl;
    // }
    // cout << "                8'd" << 60 << " : begin" << endl;
    // cout << "                    if (proj_u1_r[0] == 0 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        for(int i = 0; i <= 59; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 59; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[60] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[60]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[61] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62] <= idle_Addr;" << endl;
    // cout << "                        for(int i = 61; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 1 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        for(int i = 0; i <= 59; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 59; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[60]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[60] <= dstFrame_lb_load_addr + 1;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[61]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= idle_Addr;" << endl;
    // cout << "                        for(int i = 61; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 0 && (proj_u1_r >= r_hsize-1 || proj_v1_r >= r_vsize-1)) begin" << endl;
    // cout << "                        for(int i = 0; i <= 59; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[60]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = 61; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]   <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else begin" << endl;
    // cout << "                        for(int i = 0; i <= 59; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[60]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 61; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                end" << endl;
    // cout << "                8'd" << 61 << " : begin" << endl;
    // cout << "                    if (proj_u1_r[0] == 0 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        for(int i = 0; i <= 60; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 60; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[61] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[61]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= idle_Addr;" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 1 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        for(int i = 0; i <= 60; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 60; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[61]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[61] <= dstFrame_lb_load_addr + 1;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62] <= idle_Addr;" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 0 && (proj_u1_r >= r_hsize-1 || proj_v1_r >= r_vsize-1)) begin" << endl;
    // cout << "                        for(int i = 0; i <= 60; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[61]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62]  <= idle_Addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else begin" << endl;
    // cout << "                        for(int i = 0; i <= 60; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[61]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= idle_Addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                end" << endl;
    // cout << "                8'd" << 62 << " : begin" << endl;
    // cout << "                    if (proj_u1_r[0] == 0 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        for(int i = 1; i <= 61; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 0; i <= 61; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[0] <= dstFrame_lb_load_addr;" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 1 && proj_u1_r < r_hsize-1 && proj_v1_r < r_vsize-1) begin" << endl;
    // cout << "                        for(int i = 0; i <= 61; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        for(int i = 1; i <= 61; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62] <= dstFrame_lb_load_addr + 1;" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[0]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else if (proj_u1_r[0] == 0 && (proj_u1_r >= r_hsize-1 || proj_v1_r >= r_vsize-1)) begin" << endl;
    // cout << "                        for(int i = 0; i <= 61; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_even_AB_r[62]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                    else begin" << endl;
    // cout << "                        for(int i = 0; i <= 61; i = i + 1) begin o_dstFrame_lb_sram_odd_AB_r[i]  <= idle_Addr; end" << endl;
    // cout << "                        o_dstFrame_lb_sram_odd_AB_r[62]  <= dstFrame_lb_load_addr;" << endl;
    // cout << "                        for(int i = 0; i <= 62; i = i + 1) begin o_dstFrame_lb_sram_even_AB_r[i] <= idle_Addr; end" << endl;
    // cout << "                    end" << endl;
    // cout << "                end" << endl;

    //dstFrame_lb_load_Q
    for (int i = 0; i <= 61; i++)
    {
        cout << "                8'd" << i << " : begin" << endl;
        cout << "                    if (proj_u1_d3[0] == 0 && proj_u1_d3 < r_hsize-1 && proj_v1_d3 < r_vsize-1) begin" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_even_QB[" << i << "];" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_u_r <= i_dstFrame_lb_sram_odd_QB[" << i << "];" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_v_r <= i_dstFrame_lb_sram_even_QB[" << i+1 << "];" << endl;
        cout << "                    end" << endl;
        cout << "                    else if (proj_u1_d3[0] == 1 && proj_u1_d3 < r_hsize-1 && proj_v1_d3 < r_vsize-1) begin" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_odd_QB[" << i << "];" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_u_r <= i_dstFrame_lb_sram_even_QB[" << i << "];" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_v_r <= i_dstFrame_lb_sram_odd_QB[" << i+1 << "];" << endl;
        cout << "                    end" << endl;
        cout << "                    else if (proj_u1_d3[0] == 0 && (proj_u1_d3 >= r_hsize-1 || proj_v1_d3 >= r_vsize-1)) begin" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_even_QB[" << i << "];" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_u_r <= 0;" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_v_r <= 0;" << endl;
        cout << "                    end" << endl;
        cout << "                    else begin" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_odd_QB[" << i << "];" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_u_r <= 0;" << endl;
        cout << "                        i_dstFrame_lb_sram_QB_v_r <= 0;" << endl;
        cout << "                    end" << endl;
        cout << "                end" << endl;
    }
    cout << "                8'd62 : begin" << endl;
    cout << "                    if (proj_u1_d3[0] == 0 && proj_u1_d3 < r_hsize-1 && proj_v1_d3 < r_vsize-1) begin" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_even_QB[62];" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_u_r <= i_dstFrame_lb_sram_odd_QB[62];" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_v_r <= i_dstFrame_lb_sram_even_QB[0];" << endl;
    cout << "                    end" << endl;
    cout << "                    else if (proj_u1_d3[0] == 1 && proj_u1_d3 < r_hsize-1 && proj_v1_d3 < r_vsize-1) begin" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_odd_QB[62];" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_u_r <= i_dstFrame_lb_sram_even_QB[62];" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_v_r <= i_dstFrame_lb_sram_odd_QB[0];" << endl;
    cout << "                    end" << endl;
    cout << "                    else if (proj_u1_d3[0] == 0 && (proj_u1_d3 >= r_hsize-1 || proj_v1_d3 >= r_vsize-1)) begin" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_even_QB[62];" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_u_r <= 0;" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_v_r <= 0;" << endl;
    cout << "                    end" << endl;
    cout << "                    else begin" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_r   <= i_dstFrame_lb_sram_odd_QB[62];" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_u_r <= 0;" << endl;
    cout << "                        i_dstFrame_lb_sram_QB_v_r <= 0;" << endl;
    cout << "                    end" << endl;
    cout << "                end" << endl;

    // ofstream fo_1("debug_check_store_index_SW.txt");
    // int count = 0;
    // for (int j = 0; j < 480; j++){
    //     for (int i = 0; i < 640; i++){
    //         if (i == 639) {
    //             fo_1 << count << endl;
    //             if (count == 62) count = 0;
    //             else count++;
    //         }
    //         else fo_1 << count << endl;
    //     }
    // }

    // for (int i = 0; i <= 61; i++){
    //     cout << "                6'd" << i << " : begin i_dstFrame_lb_sram_QB_r <= i_dstFrame_lb_sram_QB[" << i << "]; end" << endl;
    // }
    
    // for (int y = 0; y < 6; y++){
    //     // for (int x = y; x < 6; x++){
    //     //     cout << "    always_ff @(posedge i_clk or negedge i_rst_n) begin" << endl;
    //     //     cout << "        if (!i_rst_n) Mat_" << x << y << "_r <= '0;" << endl;
    //     //     cout << "        else if (i_frame_end_icp)  Mat_" << x << y << "_r = Mat_" << x << y << "_r + i_ICP_Mat_" << x << y << ";" << endl;
    //     //     cout << "        else if (i_frame_end_rgbd) Mat_" << x << y << "_r = Mat_" << x << y << "_r + i_Rgbd_Mat_" << x << y << ";" << endl;
    //     //     cout << "        else Mat_" << x << y << "_r <= Mat_" << x << y << "_r;" << endl;
    //     //     cout << "    end" << endl;
    //     //     cout << endl;
    //     // }
    //     cout << "    always_ff @(posedge i_clk or negedge i_rst_n) begin" << endl;
    //     cout << "        if (!i_rst_n) Vec_" << y << "_r <= '0;" << endl;
    //     cout << "        else if (i_frame_end_icp)  Vec_" << y << "_r = Vec_" << y << "_r + i_ICP_Vec_" << y << ";" << endl;
    //     cout << "        else if (i_frame_end_rgbd) Vec_" << y << "_r = Vec_" << y << "_r + i_Rgbd_Vec_" << y << ";" << endl;
    //     cout << "        else Vec_" << y << "_r <= Vec_" << y << "_r;" << endl;
    //     cout << "    end" << endl;
    //     cout << endl;
    // }
}