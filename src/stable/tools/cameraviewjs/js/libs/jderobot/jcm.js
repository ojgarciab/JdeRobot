// **********************************************************************
//
// Copyright (c) 2003-2014 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************
//
// Ice version 3.5.1
//
// <auto-generated>
//
// Generated from file `jcm.ice'
//
// Warning: do not edit this file.
//
// </auto-generated>
//

(function(global, r)
{
    var require = typeof(r) === "function" ? r : function(){};
    require("Ice/Struct");
    require("Ice/Long");
    require("Ice/HashMap");
    require("Ice/HashUtil");
    require("Ice/ArrayUtil");
    require("Ice/StreamHelpers");
    
    var Ice = global.Ice || {};
    var jderobot = global.jderobot ||  {};

    jderobot.FQExecutableName = Slice.defineStruct(
        function(executable, host)
        {
            this.executable = executable !== undefined ? executable : null;
            this.host = host !== undefined ? host : null;
        },
        true,
        function(__os)
        {
            __os.writeString(this.executable);
            __os.writeString(this.host);
        },
        function(__is)
        {
            this.executable = __is.readString();
            this.host = __is.readString();
        },
        2, 
        false);

    jderobot.FQComponentName = Slice.defineStruct(
        function(platform, component)
        {
            this.platform = platform !== undefined ? platform : null;
            this.component = component !== undefined ? component : null;
        },
        true,
        function(__os)
        {
            __os.writeString(this.platform);
            __os.writeString(this.component);
        },
        function(__is)
        {
            this.platform = __is.readString();
            this.component = __is.readString();
        },
        2, 
        false);

    jderobot.FQInterfaceName = Slice.defineStruct(
        function(platform, component, iface)
        {
            this.platform = platform !== undefined ? platform : null;
            this.component = component !== undefined ? component : null;
            this.iface = iface !== undefined ? iface : null;
        },
        true,
        function(__os)
        {
            __os.writeString(this.platform);
            __os.writeString(this.component);
            __os.writeString(this.iface);
        },
        function(__is)
        {
            this.platform = __is.readString();
            this.component = __is.readString();
            this.iface = __is.readString();
        },
        3, 
        false);

    jderobot.FQTopicName = Slice.defineStruct(
        function(platform, component, iface, topic)
        {
            this.platform = platform !== undefined ? platform : null;
            this.component = component !== undefined ? component : null;
            this.iface = iface !== undefined ? iface : null;
            this.topic = topic !== undefined ? topic : null;
        },
        true,
        function(__os)
        {
            __os.writeString(this.platform);
            __os.writeString(this.component);
            __os.writeString(this.iface);
            __os.writeString(this.topic);
        },
        function(__is)
        {
            this.platform = __is.readString();
            this.component = __is.readString();
            this.iface = __is.readString();
            this.topic = __is.readString();
        },
        4, 
        false);

    jderobot.ProvidedInterface = Slice.defineStruct(
        function(name, id)
        {
            this.name = name !== undefined ? name : null;
            this.id = id !== undefined ? id : null;
        },
        true,
        function(__os)
        {
            __os.writeString(this.name);
            __os.writeString(this.id);
        },
        function(__is)
        {
            this.name = __is.readString();
            this.id = __is.readString();
        },
        2, 
        false);

    jderobot.RequiredInterface = Slice.defineStruct(
        function(name, id)
        {
            this.name = name !== undefined ? name : null;
            this.id = id !== undefined ? id : null;
        },
        true,
        function(__os)
        {
            jderobot.FQInterfaceName.write(__os, this.name);
            __os.writeString(this.id);
        },
        function(__is)
        {
            this.name = jderobot.FQInterfaceName.read(__is);
            this.id = __is.readString();
        },
        4, 
        false);
    Slice.defineSequence(jderobot, "ProvidesInterfacesHelper", "jderobot.ProvidedInterface", false);
    Slice.defineSequence(jderobot, "RequiresInterfacesHelper", "jderobot.RequiredInterface", false);

    jderobot.ComponentData = Slice.defineStruct(
        function(name, provides, requires)
        {
            this.name = name !== undefined ? name : null;
            this.provides = provides !== undefined ? provides : null;
            this.requires = requires !== undefined ? requires : null;
        },
        true,
        function(__os)
        {
            jderobot.FQComponentName.write(__os, this.name);
            jderobot.ProvidesInterfacesHelper.write(__os, this.provides);
            jderobot.RequiresInterfacesHelper.write(__os, this.requires);
        },
        function(__is)
        {
            this.name = jderobot.FQComponentName.read(__is);
            this.provides = jderobot.ProvidesInterfacesHelper.read(__is);
            this.requires = jderobot.RequiresInterfacesHelper.read(__is);
        },
        4, 
        false);
    global.jderobot = jderobot;
}
(typeof (global) === "undefined" ? window : global, typeof (require) === "undefined" ? undefined : require));
