# SPDX-FileCopyrightText: © 2024 Claudio Cicconetti <c.cicconetti@iit.cnr.it>
# SPDX-License-Identifier: MIT

import logging
from concurrent import futures
import sys
import argparse
import uuid
from time import sleep

import grpc

import services_pb2_grpc
import messages_pb2

from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2

logger = logging.getLogger(__name__)


class HostServicer(services_pb2_grpc.GuestAPIHost):
    def Cast(self, request, context):
        logger.info(
            "cast() originator node_id {} function_id {}, alias = {}, msg = {}".format(
                request.originator.node_id,
                request.originator.function_id,
                request.alias,
                request.msg,
            )
        )
        return google_dot_protobuf_dot_empty__pb2.Empty()

    def CastRaw(self, request, context):
        logger.info(
            "cast-raw() originator node_id {} function_id {}, dst node_id = {}, function_id = {}, msg = {}".format(
                request.originator.node_id,
                request.originator.function_id,
                request.dst.node_id,
                request.dst.function_id,
                request.msg,
            )
        )
        return google_dot_protobuf_dot_empty__pb2.Empty()

    def Call(self, request, context):
        logger.info(
            "call() originator node_id {} function_id {}, alias = {}, msg = {}".format(
                request.originator.node_id,
                request.originator.function_id,
                request.alias,
                request.msg,
            )
        )
        return messages_pb2.CallReturn(
            type=messages_pb2.CALL_RET_REPLY, msg=request.msg
        )

    def CallRaw(self, request, context):
        logger.info(
            "call-raw() originator node_id {} function_id {}, dst node_id = {}, function_id = {}, msg = {}".format(
                request.originator.node_id,
                request.originator.function_id,
                request.dst.node_id,
                request.dst.function_id,
                request.msg,
            )
        )
        return messages_pb2.CallReturn(
            type=messages_pb2.CALL_RET_REPLY, msg=request.msg
        )

    def TelemetryLog(self, request, context):
        logger.info(
            "telemetry-log() originator node_id {} function_id {}, level {}, target {}, msg {}".format(
                request.originator.node_id,
                request.originator.function_id,
                request.log_level,
                request.target,
                request.msg,
            )
        )
        return google_dot_protobuf_dot_empty__pb2.Empty()

    def Slf(self, request, context):
        return messages_pb2.InstanceId(
            node_id=uuid.UUID(int=0), function_id=uuid.UUID(int=0)
        )

    def DelayedCast(self, request, context):
        logger.info(
            "delayed-cast() originator node_id {} function_id {}, alias = {}, msg = {}, delay = {} ms".format(
                request.originator.node_id,
                request.originator.function_id,
                request.alias,
                request.msg,
                request.delay,
            )
        )
        return google_dot_protobuf_dot_empty__pb2.Empty()

    def Sync(self, request, context):
        logger.info(
            "sync() originator node_id {} function_id {}, msg size {} bytes".format(
                request.originator.node_id,
                request.originator.function_id,
                len(request.serialized_state),
            )
        )
        return google_dot_protobuf_dot_empty__pb2.Empty()


class NodeEmulator:
    def __init__(
        self,
        host_endpoint: str,
        function_endpoint: str,
        max_workers: int,
        init_payload: str,
        serialized_state: bytes,
    ) -> None:
        """Create a node emulator exposing a GuestAPIHost server and using a GuestAPIFunction client"""

        # Create the server
        logger.info(
            "starting server at {} with max {} workers".format(
                host_endpoint, max_workers
            )
        )
        self.instance_id = messages_pb2.InstanceId(
            node_id=str(uuid.uuid4()), function_id=str(uuid.uuid4())
        )

        self.server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
        services_pb2_grpc.add_GuestAPIHostServicer_to_server(
            HostServicer(),
            self.server,
        )
        self.server.add_insecure_port(host_endpoint)
        self.server.start()
        sleep(0.1)  # make sure server is started when we issue the boot() command below

        # Create the client
        logger.info(
            "starting a client towards remote host at {} FOOOOOOOOO".format(function_endpoint)
        )
        channel = grpc.insecure_channel(function_endpoint)
        self.client = services_pb2_grpc.GuestAPIFunctionStub(channel)

        # Call boot()
        self.client.Boot(
            messages_pb2.BootData(
                guest_api_host_endpoint="http://{}/".format(host_endpoint),
                instance_id=self.instance_id,
            )
        )

        # Call init()
        self.client.Init(
            messages_pb2.FunctionInstanceInit(
                init_payload=init_payload, serialized_state=serialized_state
            )
        )

    def cast(self, msg: bytes) -> str:
        self.client.Cast(
            messages_pb2.InputEventData(
                src=messages_pb2.InstanceId(
                    node_id=str(uuid.uuid4()), function_id=str(uuid.uuid4())
                ),
                msg=msg,
            )
        )

    def call(self, msg: bytes) -> str:
        reply = self.client.Call(
            messages_pb2.InputEventData(
                src=messages_pb2.InstanceId(
                    node_id=str(uuid.uuid4()), function_id=str(uuid.uuid4())
                ),
                msg=msg,
            )
        )
        if reply.type == messages_pb2.CALL_RET_REPLY:
            return reply.msg
        elif reply.type == messages_pb2.CALL_RET_ERR:
            return "err"
        return ""

    def stop(self) -> None:
        self.client.Stop(google_dot_protobuf_dot_empty__pb2.Empty())

    def wait(self) -> None:
        self.server.wait_for_termination()


if __name__ == "__main__":
    logging.basicConfig(stream=sys.stderr)
    logging.getLogger(__name__).setLevel(logging.DEBUG)

    parser = argparse.ArgumentParser(
        "Run an EDGELESS node CLI",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--log-level", type=str, default="WARNING", help="Set the log level"
    )
    parser.add_argument(
        "--host-endpoint",
        type=str,
        default="localhost:50050",
        help="Server end-point (address:port)",
    )
    parser.add_argument(
        "--function-endpoint",
        type=str,
        default="localhost:7101",
        help="Endpoint (address:port) of the function instance",
    )
    parser.add_argument(
        "--max-workers",
        type=int,
        default=10,
        help="Maximum number of workers",
    )
    parser.add_argument(
        "--init-payload", type=str, default="", help="Payload to be passed to init()"
    )
    parser.add_argument(
        "--serialized-state",
        type=str,
        default="",
        help="Serialized state to be passed to init()",
    )
    args = parser.parse_args()

    node_emulator = NodeEmulator(
        host_endpoint=args.host_endpoint,
        function_endpoint=args.function_endpoint,
        max_workers=args.max_workers,
        init_payload=args.init_payload,
        serialized_state=bytes(args.serialized_state, encoding="utf8"),
    )

    print_help = lambda: print(
        """commands:
                  help         show this help
                  cast MSG     send cast() with given payload 
                  call MSG     send call() with given payload
                  quit         send a stop() command and quit"""
    )

    print_help()
    for line in sys.stdin:
        line = line.rstrip()
        if "quit" == line.lower():
            node_emulator.stop()
            break
        elif len(line) > 5 and "cast " == line[0:5].lower():
            node_emulator.cast(bytes(line[5:], encoding="utf8"))
        elif len(line) > 5 and "call " == line[0:5].lower():
            print(
                "reply: {}".format(node_emulator.call(bytes(line[5:], encoding="utf8")))
            )
        elif "help" == line.lower():
            print_help()
        else:
            print("unknown command: {}".format(line))
