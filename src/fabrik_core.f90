module fabrik_core
  use, intrinsic :: iso_c_binding, only: c_float, c_int
  use, intrinsic :: iso_fortran_env, only: real32
  use, intrinsic :: ieee_arithmetic, only: ieee_is_finite
  use fabrik_status_codes
  implicit none
  private

  ! Re-exported so `use fabrik_core, only: FABRIK_OK` keeps working: the codes
  ! have one definition now, in fabrik_status_codes.
  public :: FABRIK_OK, FABRIK_INVALID_ARGUMENT, FABRIK_UNREACHABLE
  public :: FABRIK_NOT_CONVERGED, FABRIK_DEGENERATE_CHAIN, FABRIK_CYCLE

  real(real32), parameter :: EPSILON = 1.0e-7_real32

  public :: solve_f32, status_string

contains

  pure function all_finite(values) result(ok)
    real(real32), intent(in) :: values(:)
    logical :: ok
    ! NaN and Inf inputs would silently propagate: `x < 0` is false for NaN, so
    ! range checks alone cannot reject them. Every caller-supplied vector is
    ! screened here and reported as FABRIK_INVALID_ARGUMENT instead.
    ok = all(ieee_is_finite(values))
  end function all_finite

  pure function distance(a, b) result(value)
    real(real32), intent(in) :: a(3), b(3)
    real(real32) :: value
    value = sqrt(sum((a - b) ** 2))
  end function distance

  pure function normalized_or_z(v) result(direction)
    real(real32), intent(in) :: v(3)
    real(real32) :: direction(3), magnitude
    magnitude = sqrt(sum(v * v))
    if (magnitude > EPSILON) then
      direction = v / magnitude
    else
      direction = [0.0_real32, 0.0_real32, 1.0_real32]
    end if
  end function normalized_or_z

  function solve_f32(joints, joint_count, lengths, target, root_anchored, &
      tolerance, max_iterations, out_joints, residual) result(status)
    integer(c_int), intent(in) :: joint_count, max_iterations
    real(c_float), intent(in) :: joints(3, joint_count), target(3), tolerance
    real(c_float), intent(in), optional :: lengths(*)
    integer(c_int), intent(in) :: root_anchored
    real(c_float), intent(out) :: out_joints(3, joint_count), residual
    integer(c_int) :: status, i, iteration
    real(real32) :: segment_lengths(joint_count - 1)
    real(real32) :: work(3, joint_count), original(3, joint_count)
    real(real32) :: total_length, radius, ratio, current_residual
    real(real32) :: base(3), direction(3)

    residual = 0.0_c_float
    if (joint_count < 2 .or. max_iterations < 1) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (.not. ieee_is_finite(tolerance) .or. tolerance < 0.0_c_float) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (.not. all_finite(reshape(joints(1:3, 1:joint_count), [3 * joint_count])) .or. &
        .not. all_finite(target)) then
      status = FABRIK_INVALID_ARGUMENT
      return
    end if
    if (present(lengths)) then
      if (.not. all_finite(lengths(1:joint_count - 1))) then
        status = FABRIK_INVALID_ARGUMENT
        return
      end if
    end if

    original = real(joints(1:3, 1:joint_count), real32)
    work = original
    out_joints = work

    if (present(lengths)) then
      segment_lengths = real(lengths(1:joint_count - 1), real32)
    else
      do i = 1, joint_count - 1
        segment_lengths(i) = distance(original(:, i), original(:, i + 1))
      end do
    end if

    if (any(segment_lengths <= EPSILON)) then
      status = FABRIK_DEGENERATE_CHAIN
      return
    end if

    current_residual = distance(work(:, joint_count), target)
    if (current_residual <= tolerance) then
      residual = current_residual
      status = FABRIK_OK
      return
    end if

    total_length = sum(segment_lengths)
    if (distance(original(:, 1), target) > total_length + tolerance) then
      if (root_anchored /= 0) then
        base = original(:, 1)
      else
        base = original(:, joint_count)
      end if
      direction = normalized_or_z(target - base)
      work = 0.0_real32
      work(:, 1) = base
      do i = 1, joint_count - 1
        work(:, i + 1) = work(:, i) + direction * segment_lengths(i)
      end do
      out_joints = work
      residual = distance(work(:, joint_count), target)
      status = FABRIK_UNREACHABLE
      return
    end if

    do iteration = 1, max_iterations
      ! FORWARD REACHING: place the tip on the target and work toward the root.
      !
      ! The name is the papers', and it is the opposite of what it looks like:
      ! Aristidou & Lasenby 2011 label this "STAGE 1: FORWARD REACHING" (the
      ! chain reaches forward, onto the target), and Aristidou, Chrysanthou &
      ! Lasenby 2015 call it the forward step / first phase. The root-ward pass
      ! below is their "STAGE 2: BACKWARD REACHING". Do not "fix" this back.
      !
      ! Each joint is placed relative to the joint BEHIND it in the chain, i.e.
      ! the one just recomputed, NOT relative to the target. Referencing the
      ! target is only correct for the last segment (where work(:,i+1) is the
      ! target itself); for every earlier joint it puts the bone at the right
      ! distance from the wrong point, and it is the reason the anchored root
      ! used to drift towards the target.
      work(:, joint_count) = target
      do i = joint_count - 1, 1, -1
        radius = distance(work(:, i), work(:, i + 1))
        if (radius <= EPSILON) cycle
        ratio = segment_lengths(i) / radius
        work(:, i) = work(:, i + 1) + (work(:, i) - work(:, i + 1)) * ratio
      end do

      ! BACKWARD REACHING: pin the root, then walk toward the tip preserving
      ! each length. 2011 calls this "STAGE 2: BACKWARD REACHING".
      ! Anchored chains MUST be re-pinned every iteration, because the forward
      ! reaching pass above has just overwritten joint 1.
      if (root_anchored /= 0) then
        work(:, 1) = original(:, 1)
      else
        work(:, 1) = work(:, joint_count)
      end if
      do i = 1, joint_count - 1
        radius = distance(work(:, i), work(:, i + 1))
        if (radius <= EPSILON) cycle
        ratio = segment_lengths(i) / radius
        work(:, i + 1) = work(:, i) + (work(:, i + 1) - work(:, i)) * ratio
      end do

      current_residual = distance(work(:, joint_count), target)
      if (current_residual <= tolerance) then
        out_joints = work
        residual = current_residual
        status = FABRIK_OK
        return
      end if
    end do

    out_joints = work
    residual = current_residual
    status = FABRIK_NOT_CONVERGED
  end function solve_f32

  pure function status_string(status) result(text)
    integer(c_int), intent(in) :: status
    character(len=24) :: text
    select case (status)
    case (FABRIK_OK)
      text = "OK"
    case (FABRIK_INVALID_ARGUMENT)
      text = "INVALID_ARGUMENT"
    case (FABRIK_UNREACHABLE)
      text = "UNREACHABLE"
    case (FABRIK_NOT_CONVERGED)
      text = "NOT_CONVERGED"
    case (FABRIK_DEGENERATE_CHAIN)
      text = "DEGENERATE_CHAIN"
    case (6_c_int)
      ! FABRIK_CYCLE, defined in fabrik_order: a dependency graph with a cycle.
      text = "CYCLE"
    case default
      text = "UNKNOWN"
    end select
  end function status_string

end module fabrik_core
