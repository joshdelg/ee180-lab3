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
    addi    $t1, $0, -4 # t1 = -4
    addi    $t2, $0, 5 # t2 = 5
    sw      $t1, ($t0)      # loc(t0) == -4
    addi    $t1, $0, 0     # reset t1 back to -4
    lw     $t1, ($t0)      # load in -4, t1 = -4
    add    $t2, $t1, $t2    # 5 - 4 # t2 = 1
    sw      $t2, 4($s0)
    sw      $s1, 0($s0)

$done:
    j       $done
    nop
    .end boot
