module fabrik_core
  use, intrinsic :: iso_c_binding, only: c_float, c_int
  use, intrinsic :: iso_fortran_env, only: real32
  implicit none
  private

  integer(c_int), parameter, public :: FABRIK_OK = 0_c_int
  integer(c_int), parameter, public :: FABRIK_INVALID_ARGUMENT = 1_c_int
  integer(c_int), parameter, public :: FABRIK_UNREACHABLE = 2_c_int
  integer(c_int), parameter, public :: FABRIK_NOT_CONVERGED = 3_c_int
  integer(c_int), parameter, public :: FABRIK_DEGENERATE_CHAIN = 4_c_int

  real(real32), parameter :: EPSILON = 1.0e-7_real32

  public :: solve_f32, status_string

contains

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
    if (joint_count < 2 .or. max_iterations < 1 .or. tolerance < 0.0_c_float) then
      status = FABRIK_INVALID_ARGUMENT
      return
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
      ! Backward: place the tip on the target and work toward the root.
      work(:, joint_count) = target
      do i = joint_count - 1, 1, -1
        radius = distance(work(:, i), target)
        if (radius <= EPSILON) cycle
        ratio = segment_lengths(i) / radius
        work(:, i) = target + (work(:, i) - target) * ratio
      end do

      ! Forward: restore the root and segment lengths toward the target.
      if (root_anchored == 0) work(:, 1) = original(:, joint_count)
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
    case default
      text = "UNKNOWN"
    end select
  end function status_string

end module fabrik_core
