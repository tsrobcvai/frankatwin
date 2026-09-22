# API reference

Everything a client program touches. Quaternions on this API are **wxyz**.

## Client

```{eval-rst}
.. autoclass:: frankatwin.remote_client.FrankaTwinClient
   :members: set_ee_target, set_gains, enable, disable, get_state, get_state_history, log_start, log_stop, log_fetch, log_save, wait_for_state, move_to_q, move_to_pose, close
```

```{eval-rst}
.. autoclass:: frankatwin.local_controller.RobotState
   :members:
```

## NUC-side controller

```{eval-rst}
.. autoclass:: frankatwin.local_controller.LocalController
   :members: start_controller, stop_controller, is_controller_alive, ensure_running, set_ee_target, set_gains, enable, disable, get_state, get_all_state, log_start, log_stop, log_fetch, log_save, move_to_q, move_to_pose, close
```

## Configuration

```{eval-rst}
.. automodule:: frankatwin.config
   :members: load_config, resolve_config_path, RobotConfig, NetworkConfig, RobotSpec, ControlConfig, PathsConfig, ResetConfig, GripperConfig, LoadConfig, CollisionConfig
```

## Quaternions

```{eval-rst}
.. automodule:: frankatwin.quat
   :members:
```

## Excitation references

```{eval-rst}
.. autofunction:: frankatwin.excitation.build_multiband_trajectory
.. autofunction:: frankatwin.excitation.build_chirp_trajectory
```

## Shared memory

```{eval-rst}
.. automodule:: frankatwin.shm_layout
   :members: ShmView, SharedMemoryAccess, layout_offsets
```
