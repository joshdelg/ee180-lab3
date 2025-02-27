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
    nop
    nop
    ori     $t0, $0, 0x3210    # safe address
    nop
    nop
    addi    $t3, $0, 0  # init t3
    nop
    ori    $t1, $0, 0 # t1 = 0
    nop
    nop
    addi   $t2, $0, 5 # t2 = 5
    nop
    nop
    sw     $t2, ($t0)   # loc(t0) = 5
    nop
    nop
    nop
    ld     $t3, ($t0)   # t3 = 5
    addi   $t1, $t1, -4      # t1 = -4
    add    $t2, $t3, $t1     # add is dep on rt in exe stage
    nop
    nop
    nop
    nop
    sw      $t2, 4($s0)
    sw      $s1, 0($s0)

$done:
    j       $done
    nop
    .end boot
