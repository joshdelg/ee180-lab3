//=============================================================================
// EE180 Lab 3
//
// Instruction fetch module. Maintains PC and updates it. Reads from the
// instruction ROM.
//=============================================================================

module instruction_fetch (
    input clk,
    input rst,
    input en,
    input jump_target, // Single bit set by decode stage if j instruction
    input [31:0] pc_id,
    input [25:0] instr_id,  // Lower 26 bits of the instruction

    output [31:0] pc,

    // @joshdelg: Bit for if taking a branch, offset to branch to
    input should_branch,
    input signed [31:0] branch_offset
);


    wire [31:0] pc_id_p4 = pc_id + 3'h4;
    wire [31:0] j_addr = {pc_id_p4[31:28], instr_id[25:0], 2'b0};
    
    // Logic to support beq/bne
    // 1. Add a new wire that grabs address from ID register
    // 2. Add a new control signal 'Branch' that defines if we're doing a conditional branch

    // wire [31:0] pc_next = (jump_target) ? j_addr : (pc + 3'h4);
    wire [31:0] pc_next = pc + 3'h4;
    wire [31:0] pc_branch = pc + branch_offset;
    wire [31:0] pc_jump = j_addr;

    wire [31:0] new_pc = (should_branch) ? pc_branch :
                            (jump_target) ? pc_jump :
                                pc_next;
    
    dffare #(32) pc_reg (.clk(clk), .r(rst), .en(en), .d(new_pc), .q(pc));

endmodule
