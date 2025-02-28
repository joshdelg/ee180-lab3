###############################################################################
# File         : movn.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'movn' instruction.
#
###############################################################################
# movz rd, rs, rt
# conditional move, move if rt is 0

    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load the base address of the status/test registers
    ori     $s1, $0, 1          # Prepare the 'done' status
    ori     $t4, $0, 0
    lui     $t0, 0xdeaf
    ori     $t0, $t0, 0xbeef    # t0 is deafbeef
    ori     $t1, $0, 0          # t1 is 0
    movz    $t2, $t0, $t4       # $t2 gets 0xdeafbeef since t4 is 0, rt == 0
    movz    $t1, $t0, $s1        # $t1 could be deadbeef but it is 0 since condition is 1
    subu    $t3, $t2, $t0       # t3 = deadbeef - deadbeef # t2 mov was successful
    sltiu   $v0, $t3, 1         # v0 if t3 < 1
    sltiu   $v1, $t1, 1         # v1 if t1 < 1 # t1 mov was unsuccessful
    and     $v0, $v0, $v1       # v0 if t3 < 1 and t1 < 1
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
