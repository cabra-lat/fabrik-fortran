program test_fabrik_core
  use, intrinsic :: iso_c_binding, only: c_int
  use, intrinsic :: iso_fortran_env, only: real32
  use fabrik_core
  implicit none

  integer :: passed
  passed = 0

  call test_convergence(passed)
  call test_unreachable_failure(passed)
  call test_degenerate_chain(passed)
  call test_determinism(passed)

  print '(a,i0,a)', 'RESULT: PASS (', passed, '/4)'
  if (passed /= 4) error stop 1

contains

  subroutine expect(condition, label, passed)
    logical, intent(in) :: condition
    character(*), intent(in) :: label
    integer, intent(inout) :: passed
    if (condition) then
      passed = passed + 1
      print '(a,a)', 'PASS  ', label
    else
      print '(a,a)', 'FAIL  ', label
    end if
  end subroutine expect

  subroutine test_convergence(passed)
    integer, intent(inout) :: passed
    real(real32) :: joints(3, 4), target(3), lengths(3), out(3, 4), residual
    integer(c_int) :: status
    joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                       1.0_real32, 0.0_real32, 0.0_real32, &
                       2.0_real32, 0.0_real32, 0.0_real32, &
                       3.0_real32, 0.0_real32, 0.0_real32], [3, 4])
    lengths = [1.0_real32, 1.0_real32, 1.0_real32]
    target = [1.0_real32, 1.0_real32, 0.0_real32]
    status = solve_f32(joints, 4_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect(status == FABRIK_OK .and. residual <= 1.0e-5_real32, &
      'reachable chain converges', passed)
  end subroutine test_convergence

  subroutine test_unreachable_failure(passed)
    integer, intent(inout) :: passed
    real(real32) :: joints(3, 3), target(3), lengths(2), out(3, 3), residual
    integer(c_int) :: status
    joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                       1.0_real32, 0.0_real32, 0.0_real32, &
                       2.0_real32, 0.0_real32, 0.0_real32], [3, 3])
    lengths = [1.0_real32, 1.0_real32]
    target = [0.0_real32, 4.0_real32, 0.0_real32]
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect(status == FABRIK_UNREACHABLE .and. residual > 1.0e-5_real32 &
      .and. abs(out(2, 3) - 2.0_real32) < 1.0e-6_real32, &
      'unreachable target fails with extended straight chain', passed)
  end subroutine test_unreachable_failure

  subroutine test_degenerate_chain(passed)
    integer, intent(inout) :: passed
    real(real32) :: joints(3, 3), target(3), lengths(2), out(3, 3), residual
    integer(c_int) :: status
    joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                       0.0_real32, 0.0_real32, 0.0_real32, &
                       1.0_real32, 0.0_real32, 0.0_real32], [3, 3])
    lengths = [0.0_real32, 1.0_real32]
    target = [0.5_real32, 0.5_real32, 0.0_real32]
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect(status == FABRIK_DEGENERATE_CHAIN .and. all(out == joints), &
      'zero-length segment is explicit and non-destructive', passed)
  end subroutine test_degenerate_chain

  subroutine test_determinism(passed)
    integer, intent(inout) :: passed
    real(real32) :: joints(3, 5), target(3), lengths(4)
    real(real32) :: first(3, 5), second(3, 5), third(3, 5)
    real(real32) :: r1, r2, r3
    integer(c_int) :: s1, s2, s3
    joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                       1.0_real32, 0.1_real32, 0.0_real32, &
                       2.0_real32, 0.4_real32, 0.1_real32, &
                       2.5_real32, 1.1_real32, 0.2_real32, &
                       3.0_real32, 1.5_real32, 0.0_real32], [3, 5])
    lengths = [1.0_real32, 1.0_real32, 1.0_real32, 1.0_real32]
    target = [3.2_real32, 0.6_real32, 0.4_real32]
    s1 = solve_f32(joints, 5_c_int, lengths, target, 1_c_int, 1.0e-5_real32, 64_c_int, first, r1)
    s2 = solve_f32(joints, 5_c_int, lengths, target, 1_c_int, 1.0e-5_real32, 64_c_int, second, r2)
    s3 = solve_f32(joints, 5_c_int, lengths, target, 1_c_int, 1.0e-5_real32, 64_c_int, third, r3)
    call expect(s1 == s2 .and. s2 == s3 .and. r1 == r2 .and. r2 == r3 &
      .and. all(first == second) .and. all(second == third), &
      'repeated solves are bit-deterministic', passed)
  end subroutine test_determinism

end program test_fabrik_core
