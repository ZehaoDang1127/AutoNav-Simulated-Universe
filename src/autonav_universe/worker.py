"""Functional-mode plug-in worker. Only public API objects are serialized."""
from __future__ import annotations

import importlib
import multiprocessing as mp
import os
import sys
import time
import traceback
from multiprocessing.connection import Connection

from .api import EpisodeContext, NavigationCommand, Observation, PluginManifest


def load_algorithm(entry_point: str):
    if os.getcwd() not in sys.path:
        sys.path.insert(0, os.getcwd())
    module_name, separator, class_name = entry_point.partition(":")
    if not separator or not module_name or not class_name:
        raise ValueError("algorithm must be module:Class")
    algorithm = getattr(importlib.import_module(module_name), class_name)()
    for method in ("reset", "step", "close"):
        if not callable(getattr(algorithm, method, None)):
            raise TypeError(f"plug-in missing {method}()")
    return algorithm


def _serve(connection: Connection, entry_point: str, context: EpisodeContext) -> None:
    algorithm = None
    try:
        algorithm = load_algorithm(entry_point)
        manifest = getattr(algorithm, "manifest", None)
        if not isinstance(manifest, PluginManifest):
            raise TypeError("plug-in must declare a PluginManifest")
        if manifest.api_version != context.api_version:
            raise ValueError("plug-in API version mismatch")
        if not manifest.algorithm_version or context.track not in manifest.supported_tracks:
            raise ValueError("plug-in does not support this track")
        if not set(manifest.required_sensors).issubset(context.capabilities):
            raise ValueError("plug-in requires unavailable sensors")
        if not 0 < manifest.requested_update_hz <= context.navigation_rate_hz:
            raise ValueError("plug-in update rate exceeds scenario navigation rate")
        algorithm.reset(context)
        connection.send(("ready", manifest))
        while True:
            request, payload = connection.recv()
            if request == "stop":
                break
            if request != "step" or not isinstance(payload, Observation):
                raise TypeError("invalid worker request")
            try:
                start = time.perf_counter()
                result = algorithm.step(payload)
                if not isinstance(result, NavigationCommand):
                    raise TypeError("step() must return NavigationCommand")
                connection.send(("command", (result, time.perf_counter()-start)))
            except BaseException:
                connection.send(("error", traceback.format_exc()))
                break
    except BaseException:
        try:
            connection.send(("error", traceback.format_exc()))
        except (BrokenPipeError, OSError):
            pass
    finally:
        if algorithm is not None:
            try:
                algorithm.close()
            except BaseException:
                pass
        connection.close()


class AlgorithmWorker:
    def __init__(self, entry_point: str, context: EpisodeContext, timeout_s: float = 2.0):
        self.timeout_s = timeout_s
        parent, child = mp.get_context("spawn").Pipe()
        self.connection = parent
        self.process = mp.get_context("spawn").Process(target=_serve, args=(child, entry_point, context))
        self.process.start()
        child.close()
        try:
            kind, self.manifest = self._receive()
            if kind != "ready":
                raise RuntimeError("invalid worker startup response")
        except BaseException:
            self.close()
            raise

    def _receive(self):
        if not self.connection.poll(self.timeout_s):
            self.close()
            raise TimeoutError("algorithm worker exceeded wall-time budget")
        try:
            kind, payload = self.connection.recv()
        except EOFError as exc:
            raise RuntimeError("algorithm worker exited") from exc
        if kind == "error":
            raise RuntimeError(payload)
        return kind, payload

    def step(self, observation: Observation) -> NavigationCommand:
        self.submit(observation)
        kind, payload = self._receive()
        if kind != "command":
            raise RuntimeError("invalid worker response")
        return payload[0]

    def submit(self, observation: Observation) -> None:
        self.connection.send(("step", observation))

    def poll(self):
        if not self.connection.poll(0):
            return None
        try:
            kind, payload = self.connection.recv()
        except EOFError as exc:
            raise RuntimeError("algorithm worker exited") from exc
        if kind == "error":
            raise RuntimeError(payload)
        if kind != "command":
            raise RuntimeError("invalid worker response")
        return payload

    def close(self) -> None:
        if self.process.is_alive():
            try:
                self.connection.send(("stop", None))
                self.process.join(0.2)
            except (BrokenPipeError, OSError):
                pass
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(1.0)
        self.connection.close()
