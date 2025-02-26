###############################################################################
# File         : ori.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'ori' instruction.
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
    add     $t0, $0, $0         # Load 0x00000000 into $t0
    ori     $t0, $t0, 0xFFFF    # Should 0 extend, $t0 = 0x0000FFFF
    lui     $t1, 0xF000
    ori     $t1, $t1, 0x0001    # $t1 = 0xF0000001
    and     $t0, $t0, $t1       # $t0 = 1 if was originally 0 extended as expected
    nop
    nop
    sw      $t0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
