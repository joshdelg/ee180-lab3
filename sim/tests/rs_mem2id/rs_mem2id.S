###############################################################################
# File         : beq.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'beq' instruction.
#
###############################################################################


    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load the base address of the status/test registers
    ori     $s1, $0, 1          # Prepare the 'done' status
    ori     $t0, $0, 0x3210    # safe address
    addi    $t1, $0, 5 # t1 = 5
    ori    $t2, $0, 0 # t2 = 0
    sw      $t1, ($t0)      # t1 = 5
    lw      $t2, ($t0)      # load in 5, t2 = 5
    addi    $t2, $t2, -4    # 5 - 4 # t2 = 1
    sw      $t2, 4($s0)
    sw      $s1, 0($s0)

$done:
    j       $done
    nop
    .end boot
