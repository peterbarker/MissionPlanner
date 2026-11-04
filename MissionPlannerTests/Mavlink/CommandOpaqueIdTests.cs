using System.Collections.Generic;
using System.Linq;
using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner;
using MissionPlanner.Comms;

namespace MissionPlannerTests.Mavlink
{
    // doCommand/doCommandInt give each command a command_opaque_id and accept only the
    // COMMAND_ACK reflecting it; firmware which predates the field reflects 0.
    [TestClass]
    public class CommandOpaqueIdTests
    {
        private const uint Sys = 7;
        private const byte Comp = 1;

        private static CommsInjection Wire(MAVLinkInterface link)
        {
            var wire = new CommsInjection();
            link.BaseStream = wire;
            // the field is a MAVLink 2 extension
            link.MAVlist[Sys, Comp].mavlinkv2 = true;
            return wire;
        }

        private static MAVLink.MAVLinkMessage Sent(IEnumerable<byte> bytes) =>
            new MAVLink.MAVLinkMessage { buffer = bytes.ToArray() };

        private static byte[] Ack(ushort command, MAVLink.MAV_RESULT result, uint id) =>
            new MAVLink.MavlinkParse().GenerateMAVLinkPacket20(MAVLink.MAVLINK_MSG_ID.COMMAND_ACK,
                new MAVLink.mavlink_command_ack_t { command = command, result = (byte)result, command_opaque_id = id },
                false, Sys, Comp);

        // a non-zero id which isn't id
        private static uint Other(uint id) => id == 1 ? 2u : 1u;

        private static bool SetMode(MAVLinkInterface link) =>
            link.doCommand(Sys, Comp, MAVLink.MAV_CMD.DO_SET_MODE, 1, 5, 0, 0, 0, 0, 0);

        [TestMethod]
        public void CommandsGetDistinctNonZeroIds()
        {
            using (var link = new MAVLinkInterface())
            {
                var wire = Wire(link);
                var ids = new List<uint>();
                wire.WriteCallback = (sender, bytes) =>
                {
                    var command = (MAVLink.mavlink_command_long_t)Sent(bytes).data;
                    ids.Add(command.command_opaque_id);
                    wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.ACCEPTED, command.command_opaque_id));
                };
                Assert.IsTrue(SetMode(link));
                Assert.IsTrue(SetMode(link));
                Assert.AreEqual(2, ids.Count);
                Assert.AreNotEqual(0u, ids[0]);
                Assert.AreNotEqual(0u, ids[1]);
                Assert.AreNotEqual(ids[0], ids[1]);
            }
        }

        [TestMethod]
        public void RetriesReuseTheId()
        {
            using (var link = new MAVLinkInterface())
            {
                var wire = Wire(link);
                var ids = new List<uint>();
                wire.WriteCallback = (sender, bytes) =>
                {
                    var command = (MAVLink.mavlink_command_long_t)Sent(bytes).data;
                    ids.Add(command.command_opaque_id);
                    // lose the first ACK so that doCommand retries
                    if (ids.Count > 1)
                        wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.ACCEPTED, command.command_opaque_id));
                };
                Assert.IsTrue(SetMode(link));
                Assert.AreEqual(2, ids.Count);
                Assert.AreNotEqual(0u, ids[0]);
                Assert.AreEqual(ids[0], ids[1]);
            }
        }

        [TestMethod]
        public void AckWithAnotherIdIsIgnored()
        {
            using (var link = new MAVLinkInterface())
            {
                var wire = Wire(link);
                wire.WriteCallback = (sender, bytes) =>
                {
                    var command = (MAVLink.mavlink_command_long_t)Sent(bytes).data;
                    // e.g. for another GCS's DO_SET_MODE
                    wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.FAILED, Other(command.command_opaque_id)));
                    wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.ACCEPTED, command.command_opaque_id));
                };
                Assert.IsTrue(SetMode(link));
            }
        }

        [TestMethod]
        public void AckWithoutIdIsMatchedByCommand()
        {
            using (var link = new MAVLinkInterface())
            {
                var wire = Wire(link);
                wire.WriteCallback = (sender, bytes) =>
                {
                    var command = (MAVLink.mavlink_command_long_t)Sent(bytes).data;
                    wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.ACCEPTED, 0));
                };
                Assert.IsTrue(SetMode(link));
            }
        }

        [TestMethod]
        public void CommandIntGetsAnIdAndIgnoresAcksWithAnotherId()
        {
            using (var link = new MAVLinkInterface())
            {
                var wire = Wire(link);
                var ids = new List<uint>();
                wire.WriteCallback = (sender, bytes) =>
                {
                    var command = (MAVLink.mavlink_command_int_t)Sent(bytes).data;
                    ids.Add(command.command_opaque_id);
                    wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.FAILED, Other(command.command_opaque_id)));
                    wire.AppendBuffer(Ack(command.command, MAVLink.MAV_RESULT.ACCEPTED, command.command_opaque_id));
                };
                Assert.IsTrue(link.doCommandInt(Sys, Comp, MAVLink.MAV_CMD.DO_REPOSITION, -1, 0, 0, 0,
                    -353632610, 1491652370, 100));
                Assert.AreEqual(1, ids.Count);
                Assert.AreNotEqual(0u, ids[0]);
            }
        }
    }
}
