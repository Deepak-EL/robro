"""Ask foxglove_bridge for the URDF's mesh assets exactly like the Foxglove app does (fetchAsset over
the websocket, subprotocol foxglove.websocket.v1); minimal websocket client, standard library only."""
import base64
import json
import os
import re
import socket
import struct

URDF = r'C:\Users\DEEPAK E L\Downloads\wcr_proto_twin\ros\wcr_proto\resource\wcr_proto.urdf'


def connect():
    s = socket.create_connection(('127.0.0.1', 8765), timeout=10)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall((f'GET / HTTP/1.1\r\nHost: localhost:8765\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
               f'Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n'
               f'Sec-WebSocket-Protocol: foxglove.sdk.v1, foxglove.websocket.v1\r\n\r\n').encode())
    hdr = b''
    while b'\r\n\r\n' not in hdr:
        hdr += s.recv(1)
    assert b' 101 ' in hdr.split(b'\r\n')[0], hdr
    return s


def recv_exact(s, n):
    b = b''
    while len(b) < n:
        c = s.recv(n - len(b))
        if not c:
            raise EOFError
        b += c
    return b


def recv_frame(s):
    b0, b1 = recv_exact(s, 2)
    n = b1 & 0x7f
    if n == 126:
        n = struct.unpack('>H', recv_exact(s, 2))[0]
    elif n == 127:
        n = struct.unpack('>Q', recv_exact(s, 8))[0]
    return b0 & 0x0f, recv_exact(s, n)


def send_text(s, text):
    data, mask = text.encode(), os.urandom(4)
    n = len(data)
    head = bytes([0x81]) + (bytes([0x80 | n]) if n < 126 else bytes([0x80 | 126]) + struct.pack('>H', n))
    s.sendall(head + mask + bytes(c ^ mask[i % 4] for i, c in enumerate(data)))


uris = sorted(set(re.findall(r'package://[^"]+', open(URDF).read())))
s = connect()
ok = 0
for i, u in enumerate(uris):
    send_text(s, json.dumps({'op': 'fetchAsset', 'uri': u, 'requestId': i}))
    while True:
        op, payload = recv_frame(s)
        if op == 2 and payload[0] == 4:                           # binary FetchAssetResponse
            req, status, elen = struct.unpack_from('<IBI', payload, 1)
            if status == 0:
                ok += 1
            else:
                print(u, 'ERROR', payload[10:10 + elen].decode())
            break
print(f'{ok}/{len(uris)} mesh assets served by foxglove_bridge')
