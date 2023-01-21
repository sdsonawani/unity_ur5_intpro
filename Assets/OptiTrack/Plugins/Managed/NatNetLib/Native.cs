/* 
Copyright � 2016 NaturalPoint Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License. 
*/


using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using NaturalPoint.NatNetLib;


// NOTE: These native structure representations are in some places incomplete
// (e.g. IntPtr to unspecified data) or untested (e.g. the force plate data
// types have not received any testing). See NatNetTypes.h for more info.
//
// This code may be unstable and is subject to change.

namespace NaturalPoint.NatNetLib
{
    internal static class NatNetConstants
    {
        public const string NatNetLibDllBaseName = "NatNetLib";
        public const CallingConvention NatNetLibCallingConvention = CallingConvention.Cdecl;

        public const int MaxModels = 2000;
        public const int MaxMarkerSets = 1000;
        public const int MaxRigidBodies = 1000;
        public const int MaxNameLength = 256;
        public const int MaxMarkers = 200;
        public const int MaxRbMarkers = 20;
        public const int MaxSkeletons = 100;
        public const int MaxSkeletonRigidBodies = 200;
        public const int MaxLabeledMarkers = 1000;
        public const int MaxUnlabeledMarkers = 1000;
        public const int MaxForcePlates = 32;
        public const int MaxDevices = 32;
        public const int MaxAnalogChannels = 32;
        public const int MaxAnalogSubframes = 30;

        public const ushort DefaultCommandPort = 1510;
        public const ushort DefaultDataPort = 1511;

        public const int Ipv4AddrStrLenMax = 16;
    }


    #region Enumerations

    internal enum NatNetError
    {
        NatNetError_OK = 0,
        NatNetError_Internal,
        NatNetError_External,
        NatNetError_Network,
        NatNetError_Other,
        NatNetError_InvalidArgument,
        NatNetError_InvalidOperation
    }


    internal enum NatNetConnectionType
    {
        NatNetConnectionType_Multicast = 0,
        NatNetConnectionType_Unicast
    }


    internal enum NatNetDataDescriptionType
    {
        NatNetDataDescriptionType_MarkerSet = 0,
        NatNetDataDescriptionType_RigidBody,
        NatNetDataDescriptionType_Skeleton,
        NatNetDataDescriptionType_ForcePlate,
        NatNetDataDescriptionType_Device,
        NatNetDataDescriptionType_Camera
    };


    internal enum NatNetVerbosity
    {
        None = 0,
        Debug,
        Info,
        Warning,
        Error
    };

    #endregion Enumerations


    #region Definition types

    [StructLayout(LayoutKind.Sequential)]
    internal struct MarkerDataVector
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)]
        public float[] Values;
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sServerDescription
    {
        [MarshalAs(UnmanagedType.U1)] public bool HostPresent;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        public string HostComputerName;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] HostComputerAddress;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        public string HostApp;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] HostAppVersion;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] NatNetVersion;

        public ulong HighResClockFrequency;

        [MarshalAs(UnmanagedType.U1)] public bool ConnectionInfoValid;

        public ushort ConnectionDataPort;

        [MarshalAs(UnmanagedType.U1)] public bool ConnectionMulticast;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4)]
        public byte[] ConnectionMulticastAddress;
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sDataDescriptions
    {
        public int DataDescriptionCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxModels)]
        public sDataDescription[] DataDescriptions;
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sDataDescription
    {
        public int DescriptionType;
        public IntPtr Description;
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sMarkerSetDescription
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        public string Name;

        public int MarkerCount;
        public IntPtr MarkerNames; // char**, "array of marker names"
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sRigidBodyDescription
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        public string Name;

        public int Id;
        public int ParentId;
        public float OffsetX;
        public float OffsetY;
        public float OffsetZ;
        public int MarkerCount;
        public IntPtr MarkerPositions; // Pointer to float[MarkerCount][3]
        public IntPtr MarkerRequiredLabels; // Pointer to int32_t[MarkerCount]
        public IntPtr MarkerNames; // char**, "array of marker names"
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sSkeletonDescription
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        public string Name;

        public int Id;
        public int RigidBodyCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxSkeletonRigidBodies)]
        public sRigidBodyDescription[] RigidBodies;
    }


    // Marshalling helper for char[][] ChannelNames in sForcePlateDescription/sDeviceDescription.
    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sChannelName
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        private string Value;

        public static implicit operator string(sChannelName source)
        {
            return source.Value;
        }

        public static implicit operator sChannelName(string source)
        {
            // Note that longer strings would be silently truncated if we didn't explicitly check this here.
            if (source.Length >= NatNetConstants.MaxNameLength)
                throw new ArgumentException("String too large for field: \"" + source + "\"");

            return new sChannelName { Value = source };
        }
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sForcePlateDescription
    {
        public int Id;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string SerialNo;

        public float Width;
        public float Length;
        public float OriginX;
        public float OriginY;
        public float OriginZ;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 12 * 12)]
        public float[] CalibrationMatrix;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 4 * 3)]
        public float[] Corners;

        public int PlateType;
        public int ChannelDataType;
        public int ChannelCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxAnalogChannels)]
        public sChannelName[] ChannelNames;
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sDeviceDescription
    {
        public int Id;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string Name;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 128)]
        public string SerialNo;

        public int DeviceType;
        public int ChannelDataType;
        public int ChannelCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxAnalogChannels)]
        public sChannelName[] ChannelNames;
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sCameraDescription
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 256)]
        public string Name;

        public float PositionX;
        public float PositionY;
        public float PositionZ;

        public float RotationX;
        public float RotationY;
        public float RotationZ;
        public float RotationW;
    }

    #endregion Definition types


    #region Data types

    [StructLayout(LayoutKind.Sequential)]
    internal struct sFrameOfMocapData
    {
        public int FrameNumber;

        public int MarkerSetCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxModels)]
        public sMarkerSetData[] MarkerSets;

        public int OtherMarkerCount;
        public IntPtr OtherMarkers; // Pointer to float[OtherMarkerCount][3]

        public int RigidBodyCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxRigidBodies)]
        public sRigidBodyData[] RigidBodies;

        public int SkeletonCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxSkeletons)]
        public sSkeletonData[] Skeletons;

        public int LabeledMarkerCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxLabeledMarkers)]
        public sMarker[] LabeledMarkers;

        public int ForcePlateCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxForcePlates)]
        public sForcePlateData[] ForcePlates;

        public int DeviceCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxDevices)]
        public sDeviceData[] Devices;

        public uint Timecode;
        public uint TimecodeSubframe;
        public double Timestamp;
        public ulong CameraMidExposureTimestamp;
        public ulong CameraDataReceivedTimestamp;
        public ulong TransmitTimestamp;
        public short Params;
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sMarkerSetData
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.MaxNameLength)]
        public string Name;

        public int MarkerCount;
        public IntPtr Markers; // Pointer to float[MarkerCount][3]
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sRigidBodyData
    {
        public int Id;
        public float X;
        public float Y;
        public float Z;
        public float QX;
        public float QY;
        public float QZ;
        public float QW;
        public float MeanError;
        public short Params;
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sSkeletonData
    {
        public int Id;
        public int RigidBodyCount;
        public IntPtr RigidBodies; // Pointer to sRigidBodyData[RigidBodyCount]
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sMarker
    {
        public int Id;
        public float X;
        public float Y;
        public float Z;
        public float Size;
        public short Params;
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sAnalogChannelData
    {
        public int FrameCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxAnalogSubframes)]
        public float[] Values;
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sForcePlateData
    {
        public int Id;
        public int ChannelCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxAnalogChannels)]
        public sAnalogChannelData[] ChannelData;

        public short Params;
    }


    [StructLayout(LayoutKind.Sequential)]
    internal struct sDeviceData
    {
        public int Id;
        public int ChannelCount;

        [MarshalAs(UnmanagedType.ByValArray, SizeConst = NatNetConstants.MaxAnalogChannels)]
        public sAnalogChannelData[] ChannelData;

        public short Params;
    }


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sNatNetClientConnectParams
    {
        public NatNetConnectionType ConnectionType;
        public ushort ServerCommandPort;
        public ushort ServerDataPort;
        public string ServerAddress;
        public string LocalAddress;
        public string MulticastAddress;
    };


    [StructLayout(LayoutKind.Sequential, CharSet = CharSet.Ansi)]
    internal struct sNatNetDiscoveredServer
    {
        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.Ipv4AddrStrLenMax)]
        public string LocalAddress;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = NatNetConstants.Ipv4AddrStrLenMax)]
        public string ServerAddress;

        public ushort ServerCommandPort;
        public sServerDescription ServerDescription;
    }

    #endregion Data types


    /// <summary>
    /// Reverse P/Invoke delegate type for <see cref="NativeMethods.NatNet_Client_SetFrameReceivedCallback"/>.
    /// </summary>
    /// <param name="pFrameOfMocapData">Native pointer to <see cref="sFrameOfMocapData"/>.</param>
    /// <param name="pUserData">User-provided context (void pointer).</param>
    [UnmanagedFunctionPointer(NatNetConstants.NatNetLibCallingConvention)]
    internal delegate void NatNetFrameReceivedCallback(IntPtr pFrameOfMocapData, IntPtr pUserData);


    /// <summary>
    /// Reverse P/Invoke delegate type for <see cref="NativeMethods.NatNet_SetLogCallback"/>.
    /// </summary>
    /// <param name="level">Log message severity.</param>
    /// <param name="pMessage">Null-terminated char* containing message text.</param>
    [UnmanagedFunctionPointer(NatNetConstants.NatNetLibCallingConvention)]
    internal delegate void NatNetLogCallback(NatNetVerbosity level, IntPtr pMessage);


    /// <summary>
    /// Reverse P/Invoke delegate type for <see cref="NativeMethods.NatNet_CreateAsyncServerDiscovery"/>.
    /// </summary>
    /// <param name="level">Log message severity.</param>
    /// <param name="pMessage">Null-terminated char* containing message text.</param>
    [UnmanagedFunctionPointer(NatNetConstants.NatNetLibCallingConvention)]
    internal delegate void NatNetServerDiscoveryCallback(sNatNetDiscoveredServer discoveredServer, IntPtr pUserContext);


    internal static class NativeMethods
    {
        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern void NatNet_GetVersion(
            [In] [Out] [MarshalAs(UnmanagedType.LPArray, SizeConst = 4)]
            byte[] version);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern void NatNet_DecodeID(int compositeId, out int entityId, out int memberId);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern void NatNet_DecodeTimecode(uint compositeId, uint timecodeSubframe, out int pOutHour,
            out int pOutMinute, out int pOutSecond, out int pOutFrame, out int pOutSubFrame);


        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern void NatNet_SetLogCallback(NatNetLogCallback pfnCallback);


        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_CreateAsyncServerDiscovery(out IntPtr discoveryHandle,
            NatNetServerDiscoveryCallback pfnCallback, IntPtr pUserContext = default,
            [MarshalAs(UnmanagedType.U1)] bool startImmediately = true);

        [DllImport(NatNetConstants.NatNetLibDllBaseName, CallingConvention = NatNetConstants.NatNetLibCallingConvention,
            CharSet = CharSet.Ansi, BestFitMapping = false, ThrowOnUnmappableChar = true)]
        public static extern NatNetError NatNet_AddDirectServerToAsyncDiscovery(IntPtr discoveryHandle,
            string serverAddress);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_StartAsyncDiscovery(IntPtr discoveryHandle);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_FreeAsyncServerDiscovery(IntPtr discoveryHandle);


        //////////////////////////////////////////////////////////////////////
        // These functions are not a supported part of the public API, and are
        // subject to change without notice.

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_Create(out IntPtr clientHandle);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_Destroy(IntPtr clientHandle);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_Connect(IntPtr clientHandle,
            ref sNatNetClientConnectParams connectParams);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_Disconnect(IntPtr clientHandle);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_SetFrameReceivedCallback(IntPtr clientHandle,
            NatNetFrameReceivedCallback pfnDataCallback);

        [DllImport(NatNetConstants.NatNetLibDllBaseName, CallingConvention = NatNetConstants.NatNetLibCallingConvention,
            CharSet = CharSet.Ansi, BestFitMapping = false, ThrowOnUnmappableChar = true)]
        public static extern NatNetError NatNet_Client_Request(IntPtr clientHandle, string request,
            out IntPtr pResponse, out int pResponseLenBytes, int timeoutMs = 1000, int numAttempts = 1);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_GetServerDescription(IntPtr clientHandle,
            out sServerDescription serverDescription);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_GetDataDescriptionList(IntPtr clientHandle,
            out IntPtr pDataDescriptions, uint descriptionTypesMask = 0xFFFFFFFF);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_SecondsSinceHostTimestamp(IntPtr clientHandle, ulong inTimestamp,
            out double pOutTimeElapsed);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Client_GetPredictedRigidBodyPose(IntPtr client, int rigidBodyIndex,
            out sRigidBodyData rigidBodyData, double dt);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetTimecode(IntPtr pFrameOfMocapData, out uint timecode,
            out uint timecodeSubframe);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetRigidBodyCount(IntPtr pFrameOfMocapData,
            out int rigidBodyCount);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetRigidBody(IntPtr pFrameOfMocapData, int rigidBodyIndex,
            out sRigidBodyData rigidBodyData);


        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetSkeletonCount(IntPtr pFrameOfMocapData, out int skeletonCount);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_Skeleton_GetId(IntPtr pFrameOfMocapData, int skeletonIndex,
            out int skeletonId);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_Skeleton_GetRigidBodyCount(IntPtr pFrameOfMocapData,
            int skeletonIndex, out int rigidBodyCount);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_Skeleton_GetRigidBody(IntPtr pFrameOfMocapData, int skeletonIndex,
            int rigidBodyIndex, out sRigidBodyData rigidBodyData);


        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetTransmitTimestamp(IntPtr pFrameOfMocapData,
            out ulong pOutTransmitTimestamp);


        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetLabeledMarkerCount(IntPtr pFrameOfMocapData,
            out int labeledMarkerCount);

        [DllImport(NatNetConstants.NatNetLibDllBaseName,
            CallingConvention = NatNetConstants.NatNetLibCallingConvention)]
        public static extern NatNetError NatNet_Frame_GetLabeledMarker(IntPtr pFrameOfMocapData, int labeledMarkerIndex,
            out sMarker labeledMarkerData);
    }
}