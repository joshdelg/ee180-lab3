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
    lui     $t1, 0xdead         # t1 = 0xdeadbeef
    ori     $t1, $t1, 0xbeef
    sw      $t1, 0($t0)         # Mem[t0] = t1
    lw      $t2, 0($t0)         # t2 = 0xdeadbeef
    subu    $t3, $t2, $t1       # Deadbeef - deadbeef, should be 0
    addi    $t3, $t3, 1         # Increment by 1, should be exactly 1
    sw      $t3, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
