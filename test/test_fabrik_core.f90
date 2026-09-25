program test_fabrik_core
  use, intrinsic :: iso_c_binding, only: c_int
  use, intrinsic :: iso_fortran_env, only: real32, int64
  use, intrinsic :: ieee_arithmetic, only: ieee_value, ieee_quiet_nan, &
    ieee_positive_inf, ieee_is_finite
  use fabrik_core
  implicit none

  integer :: passed, failures
  passed = 0
  failures = 0

  call test_convergence(passed, failures)
  call test_unreachable_failure(passed, failures)
  call test_degenerate_chain(passed, failures)
  call test_determinism(passed, failures)
  call test_non_finite_input(passed, failures)
  call test_minimum_chain(passed, failures)
  call test_randomized_chains(passed, failures)

  ! List-directed output: these summaries mix words and counts, and hand-written
  ! format strings silently mismatched twice while debugging this suite.
  print *, 'RESULT: ', merge('PASS', 'FAIL', failures == 0), &
    ' (', passed, ' assertions passed, ', failures, ' failed)'
  if (failures /= 0) error stop 1

contains

  subroutine expect(condition, label, passed, failures)
    logical, intent(in) :: condition
    character(*), intent(in) :: label
    integer, intent(inout) :: passed, failures
    if (condition) then
      passed = passed + 1
      print '(a,a)', 'PASS  ', label
    else
      failures = failures + 1
      print '(a,a)', 'FAIL  ', label
    end if
  end subroutine expect

  ! Status assertions print the value they actually got, so a regression says
  ! which code came back instead of only that the expectation was missed.
  subroutine expect_status(actual, expected, label, passed, failures)
    integer(c_int), intent(in) :: actual, expected
    character(*), intent(in) :: label
    integer, intent(inout) :: passed, failures
    if (actual == expected) then
      passed = passed + 1
      print '(a,a)', 'PASS  ', label
    else
      failures = failures + 1
      print '(a,a,a,i0,a,i0,a,a)', 'FAIL  ', label, ' got status ', actual, &
        ', expected ', expected, ' (', status_string(actual), ')'
    end if
  end subroutine expect_status

  subroutine test_convergence(passed, failures)
    integer, intent(inout) :: passed, failures
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
      'reachable chain converges', passed, failures)
  end subroutine test_convergence

  subroutine test_unreachable_failure(passed, failures)
    integer, intent(inout) :: passed, failures
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
      'unreachable target fails with extended straight chain', passed, failures)
  end subroutine test_unreachable_failure

  subroutine test_degenerate_chain(passed, failures)
    integer, intent(inout) :: passed, failures
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
      'zero-length segment is explicit and non-destructive', passed, failures)
  end subroutine test_degenerate_chain

  subroutine test_determinism(passed, failures)
    integer, intent(inout) :: passed, failures
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
      'repeated solves are bit-deterministic', passed, failures)
  end subroutine test_determinism

  ! Non-finite inputs used to propagate silently: `tolerance < 0` is false for
  ! NaN, so a NaN target walked straight through the solver.
  subroutine test_non_finite_input(passed, failures)
    integer, intent(inout) :: passed, failures
    real(real32) :: joints(3, 3), lengths(2), out(3, 3), residual
    real(real32) :: target(3), nan_value, inf_value
    integer(c_int) :: status
    nan_value = ieee_value(0.0_real32, ieee_quiet_nan)
    inf_value = ieee_value(0.0_real32, ieee_positive_inf)
    joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                       1.0_real32, 0.0_real32, 0.0_real32, &
                       2.0_real32, 0.0_real32, 0.0_real32], [3, 3])
    lengths = [1.0_real32, 1.0_real32]
    target = [1.0_real32, 1.0_real32, 0.0_real32]

    out = 0.0_real32
    target(2) = nan_value
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect_status(status, FABRIK_INVALID_ARGUMENT, 'NaN target is rejected', passed, failures)

    out = 0.0_real32
    target(2) = inf_value
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect_status(status, FABRIK_INVALID_ARGUMENT, 'Inf target is rejected', passed, failures)

    out = 0.0_real32
    joints(1, 2) = nan_value
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect_status(status, FABRIK_INVALID_ARGUMENT, 'NaN joint is rejected', passed, failures)

    out = 0.0_real32
    lengths(1) = nan_value
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      1.0e-5_real32, 64_c_int, out, residual)
    call expect_status(status, FABRIK_INVALID_ARGUMENT, 'NaN segment length is rejected', passed, failures)

    out = 0.0_real32
    lengths(1) = 1.0_real32
    status = solve_f32(joints, 3_c_int, lengths, target, 1_c_int, &
      nan_value, 64_c_int, out, residual)
    call expect_status(status, FABRIK_INVALID_ARGUMENT, 'NaN tolerance is rejected', passed, failures)

    call expect(all(ieee_is_finite(out)), 'rejected input leaves finite output', passed, failures)
  end subroutine test_non_finite_input

  subroutine test_minimum_chain(passed, failures)
    integer, intent(inout) :: passed, failures
    real(real32) :: one_joint(3, 1), one_length(1), out(3, 1), residual
    real(real32) :: two_joints(3, 2), target(3), two_out(3, 2)
    integer(c_int) :: status
    one_joint = reshape([0.0_real32, 0.0_real32, 0.0_real32], [3, 1])
    one_length = [1.0_real32]
    status = solve_f32(one_joint, 1_c_int, one_length, [1.0_real32, 0.0_real32, 0.0_real32], &
      1_c_int, 1.0e-5_real32, 64_c_int, out, residual)
    call expect_status(status, FABRIK_INVALID_ARGUMENT, 'single-joint chain is rejected', passed, failures)

    two_joints = reshape([0.0_real32, 0.0_real32, 0.0_real32, &
                           1.0_real32, 0.0_real32, 0.0_real32], [3, 2])
    status = solve_f32(two_joints, 2_c_int, [1.0_real32], [2.0_real32, 0.0_real32, 0.0_real32], &
      1_c_int, 1.0e-5_real32, 64_c_int, two_out, residual)
    call expect(status == FABRIK_UNREACHABLE, 'two-joint chain reaches the reachable radius', passed, failures)
  end subroutine test_minimum_chain

  ! Property test: for deterministic pseudo-random chains, either the target is
  ! within reach (status OK and residual inside tolerance) or the status is
  ! UNREACHABLE, and in every case the segment lengths are preserved.
  subroutine test_randomized_chains(passed, failures)
    integer, intent(inout) :: passed, failures
    integer, parameter :: trials = 200, max_joints = 12
    integer :: trial, n, i, seed_state, ok_count, unreachable_count, unconverged_count
    real(real32) :: joints(3, max_joints), lengths(max_joints), out(3, max_joints)
    real(real32) :: target(3), residual, segment, expected, reach
    integer(c_int) :: status
    ok_count = 0
    unreachable_count = 0
    unconverged_count = 0
    seed_state = 20260925
    do trial = 1, trials
      n = 2 + mod(int(next_random(seed_state) * real(max_joints - 1, real32)), max_joints - 1)
      do i = 1, n
        joints(:, i) = [next_random(seed_state), next_random(seed_state), &
                        next_random(seed_state)] * 2.0_real32 - 1.0_real32
      end do
      do i = 1, n - 1
        lengths(i) = 0.25_real32 + next_random(seed_state)
      end do
      target = [next_random(seed_state), next_random(seed_state), &
                next_random(seed_state)] * 4.0_real32 - 2.0_real32
      status = solve_f32(joints, int(n, c_int), lengths(1:n - 1), target, 1_c_int, &
        1.0e-4_real32, 128_c_int, out, residual)
      if (.not. all(ieee_is_finite(reshape(out, [3 * n])))) then
        failures = failures + 1
        print '(a,i0)', 'FAIL  randomized chain produced non-finite output (trial ', trial, ')'
        return
      end if
      do i = 1, n - 1
        segment = sqrt(sum((out(:, i + 1) - out(:, i)) ** 2))
        expected = lengths(i)
        if (abs(segment - expected) > 1.0e-3_real32 * max(1.0_real32, expected)) then
          failures = failures + 1
          print '(a,i0)', 'FAIL  randomized chain changed segment length (trial ', trial, ')'
          return
        end if
      end do
      reach = sum(lengths(1:n - 1))
      if (status == FABRIK_OK) then
        if (residual > 1.0e-4_real32) then
          failures = failures + 1
          print '(a,i0)', 'FAIL  randomized chain reported OK above tolerance (trial ', trial, ')'
          return
        end if
        if (distance_to(target, out(:, n)) > reach) then
          failures = failures + 1
          print '(a,i0)', 'FAIL  randomized chain reached beyond its length (trial ', trial, ')'
          return
        end if
        ok_count = ok_count + 1
      else if (status == FABRIK_UNREACHABLE) then
        if (distance_to(target, out(:, 1)) <= reach) then
          failures = failures + 1
          print '(a,i0)', 'FAIL  chain reported UNREACHABLE for a reachable target (trial ', trial, ')'
          return
        end if
        unreachable_count = unreachable_count + 1
      else if (status == FABRIK_NOT_CONVERGED) then
        if (residual <= 1.0e-4_real32) then
          failures = failures + 1
          print '(a,i0)', 'FAIL  chain reported NOT_CONVERGED within tolerance (trial ', trial, ')'
          return
        end if
        unconverged_count = unconverged_count + 1
      else
        failures = failures + 1
        print '(a,i0,a,i0)', 'FAIL  randomized chain returned status ', status, ' (trial ', trial, ')'
        return
      end if
    end do
    call expect(ok_count > 0 .and. unreachable_count > 0 .and. unconverged_count > 0, &
      'randomized chains: statuses self-consistent, lengths preserved', passed, failures)
    print *, '  (randomized ', trials, ' chains: ', ok_count, ' OK, ', &
      unconverged_count, ' not-converged, ', unreachable_count, ' unreachable)'
  end subroutine test_randomized_chains

  function next_random(state) result(value)
    integer, intent(inout) :: state
    real(real32) :: value
    ! Deterministic 31-bit LCG; the property test must be reproducible.
    ! The multiply is done in int64 so it cannot overflow default integers.
    state = int(mod(1103515245_int64 * int(state, int64) + 12345_int64, &
      2147483647_int64))
    value = real(state, real32) / 2147483647.0_real32
  end function next_random

  pure function distance_to(a, b) result(value)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: value
    value = sqrt(sum((a - b) ** 2))
  end function distance_to

end program test_fabrik_core
