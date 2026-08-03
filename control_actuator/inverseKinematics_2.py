"""Compatibility import for the former alternate kinematics implementation."""

from .inverseKinematics import StewartPlatformIK, inv_kinematics

__all__ = ['StewartPlatformIK', 'inv_kinematics']
