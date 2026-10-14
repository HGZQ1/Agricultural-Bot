"""One spawned process for bounded status-only Gazebo RPCs (no ROS/control)."""
import json
import multiprocessing
import pickle
import time


class NativeStatusRPCUnavailable(RuntimeError):
    """Only unarmed prewarming may retry an absent/discovering service."""


def _status_worker(requests, replies, partition, service):
    try:
        from gz.transport13 import Node, NodeOptions
        from gz.msgs10.stringmsg_pb2 import StringMsg
        options = NodeOptions();options.partition = partition
        node = Node(options)
        if node.options.partition != partition:
            raise RuntimeError('native status child partition mismatch')
        replies.send({'kind':'ready'})
        while True:
            request = requests.recv()
            if request.get('op') != 'query':
                break
            packet = {'kind':'result','seq':request['seq'],
                'submitted_wall':request['submitted_wall'],
                'submitted_sim_ns':request['submitted_sim_ns']}
            try:
                ok, reply = node.request(service,StringMsg(data='{"op":"status"}'),
                                          StringMsg,StringMsg,250)
                if not ok:
                    packet['unavailable'] = True
                    raise RuntimeError('ideal steering system status unavailable')
                # Keep the entire pipe frame small enough for an atomic write;
                # parent poll(0)/recv cannot wait for a huge partial payload.
                if len(reply.data.encode('utf-8')) > 2048:
                    raise RuntimeError('native status reply exceeds bounded pipe payload')
                packet['data'] = json.loads(reply.data)
            except Exception as error:
                packet['error'] = str(error)[:256]
            packet['completed_wall'] = time.monotonic()
            if len(pickle.dumps(packet,protocol=pickle.DEFAULT_PROTOCOL)) > 3500:
                packet.pop('data',None)
                packet['error'] = 'native status result exceeds atomic pipe frame bound'
            replies.send(packet)
    except (EOFError,BrokenPipeError):
        pass
    except Exception as error:
        try:replies.send({'kind':'startup_error','error':str(error)[:256]})
        except (EOFError,BrokenPipeError,OSError):pass
    finally:
        requests.close();replies.close()


class NativeStatusProcess:
    """Single worker, single in-flight query, no queue/retry/cache renewal."""
    def __init__(self, partition, service, *, _worker=_status_worker):
        context = multiprocessing.get_context('spawn')
        # Two one-way OS pipes, not a duplex stream socket. Small frames are
        # written atomically below PIPE_BUF; poll+recv cannot see half a frame.
        child_requests, self.requests = context.Pipe(duplex=False)
        self.pipe, child_replies = context.Pipe(duplex=False)
        self.process = context.Process(target=_worker,
            args=(child_requests,child_replies,partition,service),daemon=True)
        self.ready, self.closed, self.inflight = False, False, None
        self.sequence = 0
        self.created_wall = time.monotonic()
        self.process.start();child_requests.close();child_replies.close()

    def submit(self, sim_ns, wall=None):
        if self.closed or not self.ready or self.inflight is not None:
            return False
        wall = time.monotonic() if wall is None else wall
        self.sequence += 1
        request = {'op':'query','seq':self.sequence,'submitted_wall':wall,
                   'submitted_sim_ns':int(sim_ns)}
        self.requests.send(request)
        self.inflight = request
        return True

    def poll(self, wall=None):
        """Consume an available small packet without waiting for an RPC."""
        if self.closed:
            return None
        wall = time.monotonic() if wall is None else wall
        while self.pipe.poll(0):
            try:packet = self.pipe.recv()
            except (EOFError,OSError) as error:
                raise RuntimeError('native status subprocess pipe closed') from error
            if packet.get('kind') == 'ready':
                self.ready = True
                continue
            if packet.get('kind') == 'startup_error':
                raise RuntimeError('native status subprocess startup failed: '+packet['error'])
            pending = self.inflight
            if packet.get('kind') != 'result' or pending is None or packet.get('seq') != pending['seq']:
                raise RuntimeError('unexpected or replayed native status subprocess result')
            if (packet.get('submitted_wall') != pending['submitted_wall'] or
                    packet.get('submitted_sim_ns') != pending['submitted_sim_ns'] or
                    not pending['submitted_wall'] <= packet.get('completed_wall',0) <= wall or
                    not 0 <= wall-pending['submitted_wall'] <= .5):
                raise RuntimeError('native status subprocess result expired or invalid timestamp')
            self.inflight = None
            if packet.get('error'):
                if packet.get('unavailable') is True:
                    raise NativeStatusRPCUnavailable('native status RPC unavailable: '+packet['error'])
                raise RuntimeError('native status RPC failed: '+packet['error'])
            return packet
        if not self.process.is_alive():
            raise RuntimeError('native status subprocess exited')
        if not self.ready and wall-self.created_wall > 5.:
            raise RuntimeError('native status subprocess startup deadline expired')
        if self.inflight is not None and wall-self.inflight['submitted_wall'] > .5:
            raise RuntimeError('native status subprocess query deadline expired')
        return None

    def close(self, *, wait=False):
        """Terminate only this owned worker; runtime fault path never joins."""
        if not self.closed:
            self.closed = True;self.inflight = None;self.pipe.close();self.requests.close()
            if self.process.is_alive():self.process.terminate()
        if wait:
            self.process.join(timeout=.2)
            if self.process.is_alive():
                self.process.kill();self.process.join(timeout=.2)
