###############################################################################
# File         : lw.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'lw' instruction.
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
    lui     $t0, 0              # Load some valid address 0x00003210 to register $t0
    ori     $t0, $t0, 0x3210

    addi    $t1, $0, -1     # t1 = -1
    sw      $0,  0($t0)     # 0 in (t0)
    ll      $t1, 0($t0)     # 0 in t1, load link on
    sw      $0,  0($t0)     # store 0, break atomicity
    addi    $t1, $t1, 1     # t1 = 1
    sc      $t1, 0($t0)     # Store fails, ($t0) = 0, $t1 = 0
    lw      $t3, 0($t0)     # t3 = 0
    add     $t3, $t3, $t1   # Sum should be 0 becuase both are 0
    addi    $t3, $t3, 1     # Sum should be exactly 1
    sw      $t3, 4($s0)     # Set the test result
    sw      $s1, 0($s0)     # Set 'done'

$done:
    j       $done
    nop

    .end boot
