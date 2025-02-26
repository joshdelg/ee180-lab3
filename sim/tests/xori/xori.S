###############################################################################
# File         : xor.S
# Project      : EE180 Lab 2: MIPS
#
# Standards/Formatting:
#   MIPS gas, soft tab, 80 column
#
# Description:
#   Test the functionality of the 'xor' instruction.
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
    addiu   $t0, $0, -1         # Set $t0 = 0xFFFFFFFF
    xori    $t0, $t0, 0x1234    # XOR with 0x1234 (should be zero-extended to 0x00001234)
    # Compute expected result directly: 0xFFFFFFFF XOR 0x00001234 = 0xFFFFEDCB
    lui     $t1, 0xFFFF         # $t1 = $t0 - 1
    ori     $t1, $t1, 0xEDCA
    subu    $t2, $t0, $t1       # $t0 - $t1 = 1
    sw      $t2, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
