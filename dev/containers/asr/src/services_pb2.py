# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# NO CHECKED-IN PROTOBUF GENCODE
# source: services.proto
# Protobuf Python Version: 5.29.0
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import runtime_version as _runtime_version
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
_runtime_version.ValidateProtobufRuntimeVersion(
    _runtime_version.Domain.PUBLIC,
    5,
    29,
    0,
    '',
    'services.proto'
)
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from google.protobuf import empty_pb2 as google_dot_protobuf_dot_empty__pb2
import messages_pb2 as messages__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0eservices.proto\x12\x0c\x65\x64geless_api\x1a\x1bgoogle/protobuf/empty.proto\x1a\x0emessages.proto2\xe3\x01\n\x10\x46unctionInstance\x12Q\n\x05Start\x12\".edgeless_api.SpawnFunctionRequest\x1a$.edgeless_api.StartComponentResponse\x12?\n\x04Stop\x12\x1f.edgeless_api.InstanceIdVariant\x1a\x16.google.protobuf.Empty\x12;\n\x05Patch\x12\x1a.edgeless_api.PatchRequest\x1a\x16.google.protobuf.Empty2\x92\x01\n\x0eNodeManagement\x12G\n\x0bUpdatePeers\x12 .edgeless_api.UpdatePeersRequest\x1a\x16.google.protobuf.Empty\x12\x37\n\x05Reset\x12\x16.google.protobuf.Empty\x1a\x16.google.protobuf.Empty2c\n\x10NodeRegistration\x12O\n\nUpdateNode\x12\x1f.edgeless_api.UpdateNodeRequest\x1a .edgeless_api.UpdateNodeResponse2k\n\x12\x44omainRegistration\x12U\n\x0cUpdateDomain\x12!.edgeless_api.UpdateDomainRequest\x1a\".edgeless_api.UpdateDomainResponse2\xee\x02\n\x10WorkflowInstance\x12P\n\x05Start\x12\".edgeless_api.SpawnWorkflowRequest\x1a#.edgeless_api.SpawnWorkflowResponse\x12\x38\n\x04Stop\x12\x18.edgeless_api.WorkflowId\x1a\x16.google.protobuf.Empty\x12<\n\x04List\x12\x16.google.protobuf.Empty\x1a\x1c.edgeless_api.WorkflowIdList\x12G\n\x07Inspect\x12\x18.edgeless_api.WorkflowId\x1a\".edgeless_api.WorkflowInstanceInfo\x12G\n\x07\x44omains\x12\x16.edgeless_api.DomainId\x1a$.edgeless_api.DomainCapabilitiesList2K\n\x12\x46unctionInvocation\x12\x35\n\x06Handle\x12\x13.edgeless_api.Event\x1a\x16.google.protobuf.Empty2\xf1\x01\n\x15ResourceConfiguration\x12Z\n\x05Start\x12+.edgeless_api.ResourceInstanceSpecification\x1a$.edgeless_api.StartComponentResponse\x12?\n\x04Stop\x12\x1f.edgeless_api.InstanceIdVariant\x1a\x16.google.protobuf.Empty\x12;\n\x05Patch\x12\x1a.edgeless_api.PatchRequest\x1a\x16.google.protobuf.Empty2\xc4\x02\n\x10GuestAPIFunction\x12\x36\n\x04\x42oot\x12\x16.edgeless_api.BootData\x1a\x16.google.protobuf.Empty\x12\x42\n\x04Init\x12\".edgeless_api.FunctionInstanceInit\x1a\x16.google.protobuf.Empty\x12<\n\x04\x43\x61st\x12\x1c.edgeless_api.InputEventData\x1a\x16.google.protobuf.Empty\x12>\n\x04\x43\x61ll\x12\x1c.edgeless_api.InputEventData\x1a\x18.edgeless_api.CallReturn\x12\x36\n\x04Stop\x12\x16.google.protobuf.Empty\x1a\x16.google.protobuf.Empty2\x9b\x04\n\x0cGuestAPIHost\x12=\n\x04\x43\x61st\x12\x1d.edgeless_api.OutputEventData\x1a\x16.google.protobuf.Empty\x12\x43\n\x07\x43\x61stRaw\x12 .edgeless_api.OutputEventDataRaw\x1a\x16.google.protobuf.Empty\x12?\n\x04\x43\x61ll\x12\x1d.edgeless_api.OutputEventData\x1a\x18.edgeless_api.CallReturn\x12\x45\n\x07\x43\x61llRaw\x12 .edgeless_api.OutputEventDataRaw\x1a\x18.edgeless_api.CallReturn\x12G\n\x0cTelemetryLog\x12\x1f.edgeless_api.TelemetryLogEvent\x1a\x16.google.protobuf.Empty\x12\x37\n\x03Slf\x12\x16.google.protobuf.Empty\x1a\x18.edgeless_api.InstanceId\x12\x45\n\x0b\x44\x65layedCast\x12\x1e.edgeless_api.DelayedEventData\x1a\x16.google.protobuf.Empty\x12\x36\n\x04Sync\x12\x16.edgeless_api.SyncData\x1a\x16.google.protobuf.Emptyb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'services_pb2', _globals)
if not _descriptor._USE_C_DESCRIPTORS:
  DESCRIPTOR._loaded_options = None
  _globals['_FUNCTIONINSTANCE']._serialized_start=78
  _globals['_FUNCTIONINSTANCE']._serialized_end=305
  _globals['_NODEMANAGEMENT']._serialized_start=308
  _globals['_NODEMANAGEMENT']._serialized_end=454
  _globals['_NODEREGISTRATION']._serialized_start=456
  _globals['_NODEREGISTRATION']._serialized_end=555
  _globals['_DOMAINREGISTRATION']._serialized_start=557
  _globals['_DOMAINREGISTRATION']._serialized_end=664
  _globals['_WORKFLOWINSTANCE']._serialized_start=667
  _globals['_WORKFLOWINSTANCE']._serialized_end=1033
  _globals['_FUNCTIONINVOCATION']._serialized_start=1035
  _globals['_FUNCTIONINVOCATION']._serialized_end=1110
  _globals['_RESOURCECONFIGURATION']._serialized_start=1113
  _globals['_RESOURCECONFIGURATION']._serialized_end=1354
  _globals['_GUESTAPIFUNCTION']._serialized_start=1357
  _globals['_GUESTAPIFUNCTION']._serialized_end=1681
  _globals['_GUESTAPIHOST']._serialized_start=1684
  _globals['_GUESTAPIHOST']._serialized_end=2223
# @@protoc_insertion_point(module_scope)
