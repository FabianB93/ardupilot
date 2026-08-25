/*
 * Minimal POSIX compatibility shim for the precompiled OpenH264 library.
 *
 * OpenH264 uses sysconf() in WelsQueryLogicalProcessInfo() to determine
 * the number of logical processors. The ESP32-S3 is dual-core.
 */

long sysconf(int name)
{
    (void)name;
    return 2;
}