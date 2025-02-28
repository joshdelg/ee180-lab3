//=============================================================================
// EE180 Lab 3
//
// Decode module. Determines what to do with an instruction.
//=============================================================================

`include "mips_defines.v"

module decode (
    input [31:0] pc,
    input [31:0] instr,
    input [31:0] rs_data_in,
    input [31:0] rt_data_in,

    output wire [4:0] reg_write_addr,
    output wire jump_branch,
    output wire jump_target,
    output wire jump_reg,
    output wire isJalr,
    output wire [31:0] jr_pc,
    output reg [3:0] alu_opcode,
    output wire [31:0] alu_op_x,
    output wire [31:0] alu_op_y,
    output wire mem_we,
    output wire [31:0] mem_write_data,
    output wire mem_read,
    output wire mem_byte,
    output wire mem_hb,
    output wire mem_signextend,
    output wire reg_we,
    output wire movn,
    output wire movz,
    output wire [4:0] rs_addr,
    output wire [4:0] rt_addr,
    output wire atomic_id,
    input  atomic_ex,
    output wire mem_sc_mask_id,
    output wire mem_sc_id,

    output wire stall,

    // forwarding from ex
    input reg_we_ex,
    input [4:0] reg_write_addr_ex,
    input [31:0] alu_result_ex,
    input mem_read_ex,

    // forwarding
    input reg_we_mem,
    input [4:0] reg_write_addr_mem,
    input [31:0] reg_write_data_mem,

    // @joshdelg For branch control
    output signed [31:0] branch_offset
);

//******************************************************************************
// instruction field
//******************************************************************************

    wire [5:0] op = instr[31:26];
    assign rs_addr = instr[25:21];
    assign rt_addr = instr[20:16];
    wire [4:0] rd_addr = instr[15:11];
    wire [4:0] shamt = instr[10:6];
    wire [5:0] funct = instr[5:0];
    wire [15:0] immediate = instr[15:0];

    wire [31:0] rs_data, rt_data;

//******************************************************************************
// branch instructions decode
//******************************************************************************

    wire isBEQ    = (op == `BEQ);
    wire isBGEZNL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZ);
    wire isBGEZAL = (op == `BLTZ_GEZ) & (rt_addr == `BGEZAL);
    wire isBGTZ   = (op == `BGTZ) & (rt_addr == 5'b00000);
    wire isBLEZ   = (op == `BLEZ) & (rt_addr == 5'b00000);
    wire isBLTZNL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZ);
    wire isBLTZAL = (op == `BLTZ_GEZ) & (rt_addr == `BLTZAL);
    wire isBNE    = (op == `BNE);
    wire isBranchLink = (isBGEZAL | isBLTZAL);


//******************************************************************************
// jump instructions decode
//******************************************************************************

    wire isJ    = (op == `J);

//******************************************************************************
// shift instruction decode
//******************************************************************************

    wire isSLL = (op == `SPECIAL) & (funct == `SLL);
    wire isSRL = (op == `SPECIAL) & (funct == `SRL);
    wire isSRA = (op == `SPECIAL) & (funct == `SRA);    // @joshdelg Implementation of SRA
    wire isSLLV = (op == `SPECIAL) & (funct == `SLLV);
    wire isSRLV = (op == `SPECIAL) & (funct == `SRLV);
    wire isSRAV = (op == `SPECIAL) & (funct == `SRAV);  // @joshdelg Implementation of SRAV

    wire isShiftImm = isSLL | isSRL | isSRA;                // @joshdelg Added isSRA
    wire isShift = isShiftImm | isSLLV | isSRLV | isSRAV;   // @joshdelg Added isSRAV

//******************************************************************************
// ALU instructions decode / control signal for ALU datapath
//******************************************************************************

    always @* begin
        casex({op, funct})
            {`ADDI, `DC6}:      alu_opcode = `ALU_ADD;
            {`ADDIU, `DC6}:     alu_opcode = `ALU_ADDU;
            {`SLTI, `DC6}:      alu_opcode = `ALU_SLT;
            {`SLTIU, `DC6}:     alu_opcode = `ALU_SLTU;
            {`ANDI, `DC6}:      alu_opcode = `ALU_AND;
            {`ORI, `DC6}:       alu_opcode = `ALU_OR;
            {`XORI, `DC6}:      alu_opcode = `ALU_XOR; // @joshdelg Implementing XORI
            {`LB, `DC6}:        alu_opcode = `ALU_ADD;
            {`LH, `DC6}:        alu_opcode = `ALU_ADD;
            {`LW, `DC6}:        alu_opcode = `ALU_ADD;
            {`LL, `DC6}:        alu_opcode = `ALU_ADD;
            {`LBU, `DC6}:       alu_opcode = `ALU_ADD;
            {`SB, `DC6}:        alu_opcode = `ALU_ADD;
            {`SW, `DC6}:        alu_opcode = `ALU_ADD;
            {`SC, `DC6}:        alu_opcode = `ALU_ADD;
            {`BEQ, `DC6}:       alu_opcode = `ALU_SUBU;
            {`BNE, `DC6}:       alu_opcode = `ALU_SUBU;
            {`SPECIAL, `ADD}:   alu_opcode = `ALU_ADD;
            {`SPECIAL, `ADDU}:  alu_opcode = `ALU_ADDU;
            {`SPECIAL, `SUB}:   alu_opcode = `ALU_SUB;
            {`SPECIAL, `SUBU}:  alu_opcode = `ALU_SUBU;
            {`SPECIAL, `AND}:   alu_opcode = `ALU_AND;
            {`SPECIAL, `OR}:    alu_opcode = `ALU_OR;
            {`SPECIAL, `NOR}:    alu_opcode = `ALU_NOR;
            {`SPECIAL, `XOR}:   alu_opcode = `ALU_XOR; // @joshdelg Added implementation of XOR
            {`SPECIAL, `MOVN}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `MOVZ}:  alu_opcode = `ALU_PASSX;
            {`SPECIAL, `SLT}:   alu_opcode = `ALU_SLT;
            {`SPECIAL, `SLTU}:  alu_opcode = `ALU_SLTU;
            {`SPECIAL, `SLL}:   alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRL}:   alu_opcode = `ALU_SRL;
            {`SPECIAL, `SRA}:   alu_opcode = `ALU_SRA; // @joshdelg Added implementation of SRA
            {`SPECIAL, `SLLV}:  alu_opcode = `ALU_SLL;
            {`SPECIAL, `SRLV}:  alu_opcode = `ALU_SRL;
            {`SPECIAL, `SRAV}:  alu_opcode = `ALU_SRA; // @joshdelg Added implementation of SRAV
            // @joshdelg Special2
            {`SPECIAL2, `MUL}:   alu_opcode = `ALU_MUL; // @joshdelg Added implementation of MUL
            // compare rs data to 0, only care about 1 operand
            {`BGTZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLEZ, `DC6}:      alu_opcode = `ALU_PASSX;
            {`BLTZ_GEZ, `DC6}: begin
                if (isBranchLink)
                    alu_opcode = `ALU_PASSY; // pass link address for mem stage
                else
                    alu_opcode = `ALU_PASSX;
            end
            // pass link address to be stored in $ra
            {`JAL, `DC6}:       alu_opcode = `ALU_PASSY;
            {`SPECIAL, `JALR}:  alu_opcode = `ALU_PASSY;
            // or immediate with 0
            {`LUI, `DC6}:       alu_opcode = `ALU_PASSY;
            default:            alu_opcode = `ALU_PASSX;
    	endcase
    end

//******************************************************************************
// Compute value for 32 bit immediate data
//******************************************************************************

    wire use_imm = &{op != `SPECIAL, op != `SPECIAL2, op != `BNE, op != `BEQ, op != `JAL}; // where to get 2nd ALU operand from: 0 for RtData, 1 for Immediate

    wire [31:0] imm_sign_extend = {{16{immediate[15]}}, immediate};
    wire [31:0] imm_upper = {immediate, 16'b0};
    wire [31:0] imm_zero_extend = {16'b0, immediate}; // @joshdelg Implement zero extension

    // @joshdelg Add additional casing for zero extension
    wire [31:0] imm = (op == `LUI) ? imm_upper : 
                        (|{op == `ORI, op == `ANDI, op == `XORI}) ? imm_zero_extend :
                            imm_sign_extend;

//******************************************************************************
// forwarding and stalling logic
//******************************************************************************
    // @bala: forward from exe
    wire forward_rs_exe = &{rs_addr == reg_write_addr_ex, rs_addr != `ZERO, reg_we_ex}; // tells us that we will eventually write the value to a register
    wire forward_rs_mem = &{rs_addr == reg_write_addr_mem, rs_addr != `ZERO, reg_we_mem}; // multiplex reg_we to both
    // @bala
    wire[31:0] mem_rs_data = forward_rs_mem ? reg_write_data_mem : rs_data_in;
    wire[31:0] exe_rs_data = forward_rs_exe ? alu_result_ex : rs_data_in;
    assign rs_data = forward_rs_exe ? exe_rs_data : mem_rs_data;


    // @bala
    wire forward_rt_exe = &{rt_addr == reg_write_addr_ex, rt_addr != `ZERO, reg_we_ex};
    wire forward_rt_mem = &{rt_addr == reg_write_addr_mem, rt_addr != `ZERO, reg_we_mem}; // if we are writing to rt earlier, 

    wire[31:0] mem_rt_data = forward_rt_mem ? reg_write_data_mem : rt_data_in;
    wire[31:0] exe_rt_data = forward_rt_exe ? alu_result_ex : rt_data_in;
    assign rt_data = forward_rt_exe ? exe_rt_data : mem_rt_data;


    // load use cases, TODO: make sure we don't stall on add 
    wire rs_mem_dependency = &{rs_addr == reg_write_addr_ex, mem_read_ex, rs_addr != `ZERO};
    // @bala
    wire rt_mem_dependency = &{rt_addr == reg_write_addr_ex, mem_read_ex, rt_addr != `ZERO}; // if we know we are reading and execute rt addr is same


    wire isLUI = op == `LUI;
    wire read_from_rs = ~|{isLUI, jump_target, isShiftImm};

    wire isALUImm = |{op == `ADDI, op == `ADDIU, op == `SLTI, op == `SLTIU, op == `ANDI, op == `ORI, op == `XORI};
    wire read_from_rt = ~|{isLUI, jump_target, isALUImm, mem_read};

    assign stall = (rs_mem_dependency | rt_mem_dependency) & read_from_rs; // TODO for rt

    assign jr_pc = rs_data;
    assign mem_write_data = rt_data;

//******************************************************************************
// Determine ALU inputs and register writeback address
//******************************************************************************

    // for shift operations, use either shamt field or lower 5 bits of rs
    // otherwise use rs

    wire [31:0] shift_amount = isShiftImm ? shamt : rs_data[4:0];
    assign alu_op_x = isShift ? shift_amount : rs_data;

    // for link operations, use next pc (current pc + 8)
    // for immediate operations, use Imm
    // otherwise use rt

    assign alu_op_y = (use_imm) ? imm : (isJal | isJalr) ? pc + 8: rt_data;
    // assign alu_op_y = (use_imm) ? imm : rt_data;
    assign reg_write_addr = (use_imm) ? rt_addr : (isJal | isJalr) ? `RA : rd_addr;
    // assign reg_write_addr = (use_imm) ? rt_addr : rd_addr;
    // determine when to write back to a register (any operation that isn't an
    // unconditional store, non-linking branch, or non-linking jump)
    assign reg_we = ~|{(mem_we & (op != `SC)), isJ, jump_reg, isBGEZNL, isBGTZ, isBLEZ, isBLTZNL, isBNE, isBEQ}; // cases we won't write!

    // determine whether a register write is conditional
    assign movn = &{op == `SPECIAL, funct == `MOVN};
    assign movz = &{op == `SPECIAL, funct == `MOVZ};

//******************************************************************************
// Memory control
//******************************************************************************
    // assign mem_we = |{op == `SW, op == `SB, (op == `SC & should_write)};    // write to memory
    assign mem_we = |{op == `SW, op == `SB, op == `SC};
    // assign mem_read = 1'b0;                     // use memory data for writing to a register
    // @joshdelg Implement lw
    assign mem_read = |{op == `LW, op == `LB, op == `LBU, op == `LH, op == `LL};
    assign mem_byte = |{op == `SB, op == `LB, op == `LBU};    // memory operations use only one byte
    assign mem_hb   = |{op == `SH, op == `LH};
    assign mem_signextend = ~|{op == `LBU};     // sign extend sub-word memory reads

//******************************************************************************
// Load linked / Store conditional
//******************************************************************************
    assign mem_sc_id = (op == `SC);

    // 'atomic_id' is high when a load-linked has not been followed by a store.
    assign atomic_id = (op == `LL); // this goes high for one cycle and passes it along to the atomic_ex
    // atomic_ex has the LL == high

    // 'mem_sc_mask_id' is high when a store conditional should not store
    // Store conditional should not store if haven't been prefixed by LL or some
    // store has already happened (so atomic ex will be low because no longer atomic)
    assign mem_sc_mask_id = ~atomic_ex & op == `SC;
    // assign mem_sc_mask_id = (atomic_ex & (op == `SW | op == `SB)); 
    // high when shouldn't store
    // atomic_ex should be high when 
    
    //if (atomic_id) -> if (mem_sc_mask_id != 0) -> rt_data = 1 or rt_data = 0
//    wire should_write =  atomic_id ? !mem_sc_mask_id ? 1 : 0 : 0;

//******************************************************************************
// Branch resolution
//******************************************************************************

    wire isEqual = rs_data == rt_data;
    // Single bit values for if this type of control flow will be taken

    // @ bala
    // bgez: when bgeznl and rs_data >= 0 
    // bgtz: when bgtz and rs_data > 0
    // blez: when rs <= 0 isBLEZ
    // bltz: when rs < 0 and isBLTZNL
    // bne: when bne and ~isEqual
    // 
    assign jump_branch = |{isBEQ & isEqual,
                           isBNE & ~isEqual,
                           isBGTZ & (rs_data > 0),
                           isBGEZNL & (rs_data >= 0),
                           isBLTZNL & ($signed(rs_data) < 0),
                           isBLEZ & ($signed(rs_data) <= 0)
                           };
    // @joshdelg Assign the target of a branch to imm
    assign branch_offset = imm <<< 2; // Arithmetic left shift

    assign jump_target = isJ | jump_reg | isJal | isJalr;
    assign isJal = op == `JAL;
    assign isJalr = &{funct == `JALR, op == 0};
    assign jump_reg = &{funct == `JR, op == 0};



endmodule
