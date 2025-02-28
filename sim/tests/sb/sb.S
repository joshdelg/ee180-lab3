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
    sw      $t1, 0($t0)         # Store 0x00000000 into addr t0
    addi    $t2, $0, 0xde       # --- 1st Byte ----
    sb      $t2, 0($t0)         # Store 0xde into addr t0 (MSByte)
    lui     $t1, 0xde00         # t1 = 0xde000000
    lw      $t3, 0($t0)         # t3 = 0xde000000
    subu    $t4, $t3, $t1       # t3 - t1 = 0
    sltiu   $t4, $t4, 1         # v0 = 1 if were equal!
    addi    $t2, $0, 0xad       # --- 2nd Byte ----
    sb      $t2, 1($t0)         # Store 0xad into addr t0 (MSByte)
    lui     $t1, 0xdead         # t1 = 0xdead0000
    lw      $t3, 0($t0)         # t3 = 0xdead0000
    subu    $t5, $t3, $t1       # t3 - t1 = 0
    sltiu   $t5, $t5, 1         # v0 = 1 if were equal!
    addi    $t2, $0, 0xbe       # --- 3rd Byte ----
    sb      $t2, 2($t0)         # Store 0xbe into addr t0 (MSByte)
    ori     $t1, 0xbe00         # t1 = 0xdeadbe00
    lw      $t3, 0($t0)         # t3 = 0xdeadbe00
    subu    $t6, $t3, $t1       # t3 - t1 = 0
    sltiu   $t6, $t6, 1         # v0 = 1 if were equal!
    addi    $t2, $0, 0xef     # ---- 4th byte ----
    sb      $t2, 3($t0)         # Store 0xbeef into lower half of t0
    ori     $t1, $t1, 0x00ef    # t1 = 0xdeadbeef
    lw      $t3, 0($t0)         # t3 = 0xdeadbeef
    subu    $t7, $t3, $t1       # t3 - t1 = 0 = v1
    sltiu   $t7, $t7, 1         # v1 = 1 if were equal!
    and     $v0, $t4, $t5       # Both v0 and v1 must be true
    and     $v1, $t6, $t7       # Both v0 and v1 must be true
    and     $v0, $v0, $v1       # Both v0 and v1 must be true
    sw      $v0, 4($s0)         # Set the test result
    sw      $s1, 0($s0)         # Set 'done'

$done:
    j       $done
    nop

    .end boot
