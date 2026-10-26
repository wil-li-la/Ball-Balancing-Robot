# Reader side of camd.py: frames of the camera broker, newest first, without polling.
#
#   cam = camclient.Camera("sonix")
#   seq, t_capture, frame = cam.read()      # blocks until a frame newer than the last one is there
#   cam.ctl("lock")                         # freeze the exposure (see camd.py)

import mmap
import os
import socket
import struct

import numpy as np

from camd import HEADER, HEADER_SIZE, MAGIC, ctl_path, shm_path, sock_path


class Camera:
    def __init__(self, name="sonix"):
        self.name = name
        try:
            self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            self.sock.connect(sock_path(name))
            fd = os.open(shm_path(name), os.O_RDONLY)
        except OSError as e:
            raise RuntimeError(f"camera broker '{name}' is not running (python3 camd.py --camera {name}): {e}")
        self.mem = mmap.mmap(fd, 0, prot=mmap.PROT_READ)
        os.close(fd)
        magic, self.width, self.height, self.channels, _, _, _ = HEADER.unpack(self.mem[:HEADER.size])
        if magic != MAGIC:
            raise RuntimeError(f"{shm_path(name)} is not a camd frame file")
        self.slot = self.width * self.height * self.channels
        self.seq = 0

    def header(self):
        _, _, _, _, seq, wseq, t = HEADER.unpack(self.mem[:HEADER.size])
        return seq, wseq, t

    def read(self):
        # the newest frame: notifications that piled up while the caller was busy are drained, like a queue of depth 1
        token = self.sock.recv(8)
        if len(token) < 8:
            raise RuntimeError(f"camera broker '{self.name}' went away")
        self.sock.setblocking(False)
        try:
            while True:
                more = self.sock.recv(8)
                if len(more) < 8:
                    raise RuntimeError(f"camera broker '{self.name}' went away")
                token = more
        except BlockingIOError:
            pass
        finally:
            self.sock.setblocking(True)
        while True:
            seq, _, t = self.header()
            off = HEADER_SIZE + (seq % 2) * self.slot
            frame = np.frombuffer(self.mem[off:off + self.slot], np.uint8).reshape(self.height, self.width, self.channels).copy()
            _, wseq_after, _ = self.header()
            if wseq_after < seq + 2:         # nobody started writing into this slot while it was copied
                self.seq = seq
                return seq, t, frame

    def ctl(self, command):
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        s.connect(ctl_path(self.name))
        f = s.makefile("rw")
        f.write(command + "\n")
        f.flush()
        reply = f.readline().strip()
        s.close()
        if reply.startswith("error"):
            raise RuntimeError(f"camd {command}: {reply}")
        return reply

    def close(self):
        self.sock.close()
        self.mem.close()
