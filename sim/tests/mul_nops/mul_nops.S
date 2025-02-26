###############################################################################
# File         : mul.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'mul' instruction.
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
    lui     $t0, 0xffff         # A = 0xfffffffd (-3)
    nop
    nop
    nop
    nop
    ori     $t0, $t0, 0xfffd
    nop
    nop
    nop
    nop
    ori     $t1, $0, 0x4        # B = 0x4
    nop
    nop
    nop
    nop
    mul     $t2, $t0, $t1       # C = A*B = -12
    nop
    nop
    nop
    nop
    mul     $t3, $t0, $t0       # D = A*A = 9
    nop
    nop
    nop
    nop
    mul     $t4, $t1, $t1       # E = B*B = 16
    nop
    nop
    nop
    nop
    addiu   $s2, $t2, 12
    nop
    nop
    nop
    nop
    sltiu   $v0, $s2, 1
    nop
    nop
    nop
    nop
    addiu   $s3, $t3, -9
    nop
    nop
    nop
    nop
    sltiu   $v1, $s3, 1
    nop
    nop
    nop
    nop
    and     $v0, $v0, $v1
    nop
    nop
    nop
    nop
    addiu   $s4, $t4, -16
    nop
    nop
    nop
    nop
    sltiu   $v1, $s4, 1
    nop
    nop
    nop
    nop
    and     $v0, $v0, $v1
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
