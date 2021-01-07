
//Part of the code that runs on bpmp r5
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "project.h"
#include "nvboot_hardware_access_int.h"
#include "arapb2jtag.h"



//DATA STRUCTURES needed
unsigned int rmw_data_array[450]; // Largest chainlength in sys cluster is 14163 which is 443 Dwords. Giving an extra head room of 7 Dwords. ARAJU : is Unsigned int 32 bits.
unsigned int InstructionA=0; unsigned int LengthA=0; unsigned int CountA=0;
unsigned int InstructionB=0; unsigned int LengthB=0; unsigned int CountB=0;
unsigned int InstructionC=0; unsigned int LengthC=0; unsigned int CountC=0;
unsigned int InstructionD=0; unsigned int LengthD=0; unsigned int CountD=0;
unsigned int Bit_Pos=0;
unsigned int Val_Bit_Pos=0;
unsigned int InstructionB_valid=0;
unsigned int InstructionC_valid=0;
unsigned int InstructionD_valid=0;

 void write_jtag_reg(unsigned int id, unsigned int length)
{
        unsigned int temp=0;
        // Write ACCESS_CONFIG with 0s to clear previous operation
        NV_WRITE32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CONFIG_0, 0);
        temp =  ((length &0x7F800)>>11 ); 
        NV_WRITE32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CONFIG_0,temp);
        // Write Cluster ID to 1, IR to id in ACCESS_CTRL, 1 to 
        temp = (1 << APB2JTAG_ACCESS_CTRL_0_CLUSTER_SEL_SHIFT)|
            ((length&0x7ff) << APB2JTAG_ACCESS_CTRL_0_REG_LENGTH_SHIFT)|
            (1 << APB2JTAG_ACCESS_CTRL_0_REQ_CTRL_SHIFT)|
            (id);
        NV_WRITE32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CTRL_0,temp);
        // Poll for CTRL_STATUS as 1 in ACCESS_CTRL
        do
        {
            temp= NV_READ32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CTRL_0);
        }while((temp & APB2JTAG_ACCESS_CTRL_0_CTRL_STATUS_FIELD) != APB2JTAG_ACCESS_CTRL_0_CTRL_STATUS_FIELD);
        return;
}
 void read_check_jtag_reg(unsigned int id, unsigned int length,  unsigned int iter)
{
        unsigned int temp=0;

        temp =  (((length &0x7F800)>>11 ))| 
            (((iter&0xFC0) >> 6)<<APB2JTAG_ACCESS_CONFIG_0_DWORD_EN_MSB_SHIFT)|
            (1<<APB2JTAG_ACCESS_CONFIG_0_BURST_SHIFT);
        NV_WRITE32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CONFIG_0,temp);
        temp = (1 << APB2JTAG_ACCESS_CTRL_0_CLUSTER_SEL_SHIFT)|
            ((length&0x7ff) << APB2JTAG_ACCESS_CTRL_0_REG_LENGTH_SHIFT)|
            ((iter&0x3F)<<APB2JTAG_ACCESS_CTRL_0_DWORD_EN_SHIFT)|
            (1 << APB2JTAG_ACCESS_CTRL_0_REQ_CTRL_SHIFT)|
            (id );
        NV_WRITE32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CTRL_0,temp);
        // Poll for CTRL_STATUS as 1 in ACCESS_CTRL
        do
        {
            temp= NV_READ32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CTRL_0);
        }while((temp & APB2JTAG_ACCESS_CTRL_0_CTRL_STATUS_FIELD) != APB2JTAG_ACCESS_CTRL_0_CTRL_STATUS_FIELD);
        return;
}

void apb2jtag_read(unsigned int instr, unsigned int length)
{
    unsigned int i; // Loop Variable
    //Write 0 to the data array
    for(i=0; i<=(length/32); i++)
        rmw_data_array[i]=0;
    
    // READ the REGISTER THAT YOU WANT TO WRITE INTO
    NV_WRITE32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_CONFIG_0, 0);
    for(i=0; i <=(length/32); i++){
        read_check_jtag_reg( instr, length , i);
        rmw_data_array[i] = NV_READ32( NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_DATA_0);
        if((length % 32) && (i == length/32))
        {
            rmw_data_array[i] = ((rmw_data_array[i]) >> (32 - (length % 32)));
        }

    }

}
inline void modify_rdata_at_bitposition_with_bitval(unsigned int Bit_Pos, unsigned int Val_Bit_Pos){
        // Write val_bit_pos to bit_pos
        rmw_data_array[Bit_Pos/32] =(Val_Bit_Pos == 1)?  (rmw_data_array[Bit_Pos/32] | (1<<(Bit_Pos%32)))  :  (rmw_data_array[Bit_Pos/32]  & (unsigned int)~((1<<(Bit_Pos%32)))); 
}
void apb2jtag_write(unsigned int instr, unsigned int length){
    unsigned int i; // Loop Variable
        //WRITE THE REGISTER BACK
        write_jtag_reg( instr,  length);
        for( i=0; i <= (length/32); i++)
        {
            NV_WRITE32(  NV_ADDRESS_MAP_APB2JTAG_BASE + APB2JTAG_ACCESS_DATA_0, rmw_data_array[i]);
        }
}


void NvBootApb2jtag_Update(unsigned int arg0, unsigned int arg1, unsigned int arg2, unsigned int arg3,unsigned int arg4, unsigned int arg5, unsigned int arg6, unsigned int arg7 )
{
    unsigned int i;unsigned int arg_ptr=0;
    unsigned int arg_array[8]={arg0,arg1,arg2,arg3,arg4,arg5,arg6,arg7};

        if(arg0 == 0)
        {
                return;
        }
        i=0;
        //Extracting InstructionA InstructionB_valid LengthA CountA
        InstructionA = (arg_array[arg_ptr] & 0xFF);
        InstructionB_valid = (arg_array[arg_ptr] & 0x80000000) >> 31;
        LengthA = (arg_array[arg_ptr] & 0x7FFF0000) >> 16;
        CountA = (arg_array[arg_ptr] & 0xF000) >> 12;
        //Reading the regiser and populating rmw_data_array
        apb2jtag_read(InstructionA,LengthA);
        //Point to next arg in the array.
        arg_ptr++;
        //Count+1 number of bits needs to be updated 
        while(i <= CountA )
        {
            // Extract the Bit position and Value to be written into that bit position from the lower word.
            Bit_Pos = (arg_array[arg_ptr] & 0x7FFF); 
            Val_Bit_Pos = (arg_array[arg_ptr] & 0x8000) >> 15;
            //Modify Bit Value into Bit position
            modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
            i++;
            //Check if number of bits to be modified are counted
            if(i > CountA)
            {
                //If odd number of bits are to be modified then update arg pointer and exit
                arg_ptr++;
                break;
            }
            else
            {
                //If even number of bits are to be modified. Read teh Bit Position and Value to be written from the upper word.
                Bit_Pos = (arg_array[arg_ptr] & 0x7FFF0000)>>16; 
                Val_Bit_Pos = (arg_array[arg_ptr] & 0x80000000) >> 31;
                //Modify Bit Value into Bit position
                modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                // Point to next arg
                arg_ptr++;
                i++;
            }

        }
      
        //Writing back the register
        apb2jtag_write(InstructionA,LengthA);

        if (InstructionB_valid == 1) { // Register B
            //Extracting InstructionB InstructionC_valid LengthB CountB
            InstructionB = (arg_array[arg_ptr] & 0xFF);
            InstructionC_valid = (arg_array[arg_ptr] & 0x80000000) >> 31;
            LengthB = (arg_array[arg_ptr] & 0x7FFF0000) >> 16;
            CountB = (arg_array[arg_ptr] & 0xF000) >> 12;
            //Reading the regiser and populating rmw_data_array
            apb2jtag_read(InstructionB,LengthB);
            arg_ptr++;

            //Extracting data bits from r1 r2.. based on Count
            i=0;
            while(i <= CountB )
            {
                Bit_Pos = (arg_array[arg_ptr] & 0x7FFF); 
                Val_Bit_Pos = (arg_array[arg_ptr] & 0x8000) >> 15;
                modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                i++;
                if(i > CountB)
                {
                    arg_ptr++;
                    break;
                }
                else
                {
                    Bit_Pos = (arg_array[arg_ptr] & 0x7FFF0000)>>16; 
                    Val_Bit_Pos = (arg_array[arg_ptr] & 0x80000000) >> 31;
                    modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                    arg_ptr++;
                    i++;
                }

            }

            //Writing back the register
            apb2jtag_write(InstructionB,LengthB);

        }

        if (InstructionC_valid == 1) { // Register C
            //Extracting InstructionC InstructionD_valid LengthC CountC
            InstructionC = (arg_array[arg_ptr] & 0xFF);
            InstructionD_valid = (arg_array[arg_ptr] & 0x80000000) >> 31; 
            LengthC = (arg_array[arg_ptr] & 0x7FFF0000) >> 16;
            CountC = (arg_array[arg_ptr] & 0xF000) >> 12;
            //Reading the regiser and populating rmw_data_array
            apb2jtag_read(InstructionC,LengthC);
            arg_ptr++;

            //Extracting data bits from r1 r2.. based on Count
            i=0;
            while(i <= CountC )
            {
                Bit_Pos = (arg_array[arg_ptr] & 0x7FFF); 
                Val_Bit_Pos = (arg_array[arg_ptr] & 0x8000) >> 15;
                modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                i++;
                if(i > CountC)
                {
                    arg_ptr++;
                    break;
                }
                else
                {
                    Bit_Pos = (arg_array[arg_ptr] & 0x7FFF0000)>>16; 
                    Val_Bit_Pos = (arg_array[arg_ptr] & 0x80000000) >> 31;
                    modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                    arg_ptr++;
                    i++;
                }

            }

            //Writing back the register
            apb2jtag_write(InstructionC,LengthC);


        }	

        if (InstructionD_valid == 1) { // Register D
            //Extracting InstructionD  LengthD CountD
            InstructionD = (arg_array[arg_ptr] & 0xFF);
            LengthD = (arg_array[arg_ptr] & 0x7FFF0000) >> 16;
            CountD = (arg_array[arg_ptr] & 0xF000) >> 12;
            //Reading the regiser and populating rmw_data_array
            apb2jtag_read(InstructionD,LengthD);
            arg_ptr++;

            //Extracting data bits from r1 r2.. based on Count
            i=0;
            while(i <= CountD )
            {
                Bit_Pos = (arg_array[arg_ptr] & 0x7FFF); 
                Val_Bit_Pos = (arg_array[arg_ptr] & 0x8000) >> 15;
                modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                i++;
                if(i > CountD)
                {
                    arg_ptr++;
                    break;
                }
                else
                {
                    Bit_Pos = (arg_array[arg_ptr] & 0x7FFF0000)>>16; 
                    Val_Bit_Pos = (arg_array[arg_ptr] & 0x80000000) >> 31;
                    modify_rdata_at_bitposition_with_bitval(Bit_Pos,Val_Bit_Pos);
                    arg_ptr++;
                    i++;
                }

            }

            //Writing back the register
            apb2jtag_write(InstructionD,LengthD);

        }

}




