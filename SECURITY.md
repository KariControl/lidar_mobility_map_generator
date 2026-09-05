# Security policy

## Supported versions

Only the latest published release receives security fixes. Pre-release builds
and locally modified maps are supported on a best-effort basis.

## Reporting a vulnerability

Use the repository's **Security** tab to submit a private vulnerability report.
If private vulnerability reporting is unavailable, open an issue containing no
vulnerability details and request a private contact channel before disclosing
the report. Never put exploit details or sensitive data in that issue.

Include the affected version, a minimal reproduction, impact, and any proposed
mitigation. Remove credentials, raw rosbag data, point clouds, exact site
coordinates, vehicle registrations, faces, and other private information.

The Web GUI is designed for a trusted workstation and binds to loopback by
default. Its request token is not remote-user authentication. Do not expose the
editor directly to an untrusted network; use an authenticated tunnel and
read-only review where appropriate.

Generated maps and offline validators are not a safety certification or an
authorization to operate a vehicle or robot. Operational hazards, incorrect
maps, and unsafe deployment conditions should be handled through the project's
safety and issue-review process after sensitive data has been removed.
