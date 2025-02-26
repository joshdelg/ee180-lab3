###############################################################################
# File         : sra.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'sra' instruction.
#
###############################################################################

    .section .boot, "x"
    .balign 4
    .set    noreorder
    .global boot
    .ent    boot
boot:
    lui     $s0, 0x8002         # Load the base address of the status/test registers
    nop
    nop
    nop
    nop
    ori     $s1, $0, 1          # Prepare the 'done' status
    nop
    nop
    nop
    nop
    lui     $t0, 0xdeaf         # A = 0xdeafbeef
    nop
    nop
    nop
    nop
    ori     $t0, $t0, 0xbeef
    nop
    nop
    nop
    nop
    sra     $t1, $t0, 4         # B = 0xdeafbeef >> 4 = 0xfdeafbee
    nop
    nop
    nop
    nop
    lui     $t2, 0xfdea         # C = 0xfdeafbee
    nop
    nop
    nop
    nop
    ori     $t2, $t2, 0xfbee
    nop
    nop
    nop
    nop
    subu    $t3, $t1, $t2       # D = B - C = 1
    nop
    nop
    nop
    nop
    sltiu   $v0, $t3, 1
    nop
    nop
    nop
    nop
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
