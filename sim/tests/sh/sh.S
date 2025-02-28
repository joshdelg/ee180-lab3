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
    addi    $t1, $0, 0          # t1 = 0x00000000
    sw      $t1, 0($t0)          # Store 0x00000000 into addr t0
    addi    $t2, $0, 0xdead     # --- Upper HW ----
    sh      $t2, 0($t0)         # Store 0xdead into addr t0 (MS2B)
    lui     $t1, 0xdead         # t1 = 0xdead0000
    lw      $t3, 0($t0)         # t3 = 0xdead0000
    subu    $v0, $t3, $t1       # t3 - t1 = 0
    sltiu   $v0, $v0, 1         # v0 = 1 if were equal!
    addi    $t2, $0, 0xbeef     # ---- Lower HW ----
    sh      $t2, 2($t0)         # Store 0xbeef into lower half of t0
    ori     $t1, $t1, 0xbeef    # t1 = 0xdeadbeef
    lw      $t3, 0($t0)         # t3 = 0xdeadbeef
    subu    $v1, $t3, $t1       # t3 - t1 = 0 = v1
    sltiu   $v1, $v1, 1         # v1 = 1 if were equal!
    and     $v0, $v0, $v1       # Both v0 and v1 must be true
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
