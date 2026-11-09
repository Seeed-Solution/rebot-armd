# Security Policy

rebot-armd controls a physical robot arm. Bugs that let a client move the arm or change its
state without holding the control lease, bypass the joint, speed, temperature or workspace
limits, defeat the lease watchdog or the E-stop latch, or cut torque away from the resting pose
are treated as security issues.

Known properties of the current design, not vulnerabilities by themselves:

- The HTTP and WebSocket API on port 8765 has no authentication. Anyone who can reach the port
  can read the arm state, request the lease (and take it over with `force`), trigger the E-stop
  and start recordings, including writing a recording to a path of their choice.
- The Debian package starts a Wi-Fi access point with the default password `12345678` unless
  `/userdata/local/apps/rebot-teleop.config.json` sets another one.

Run the arm on a network you trust and change the access point password.

## Reporting a vulnerability

Please do not open a public issue for a vulnerability.

- Use GitHub private vulnerability reporting: the **Security** tab of
  https://github.com/Seeed-Solution/rebot-armd, then **Report a vulnerability**.
- If that form is not available, open an issue that only asks for a private contact,
  without technical details.

Include the affected version or commit, the setup (follower model, profile, how armd was
started, client used), steps to reproduce, and the effect you observed.

The report is kept private until a fix is available on the default branch.

## Supported versions

Only the latest commit on the default branch is supported.

## Scope

Issues in the host SDK and MCP server belong to https://github.com/Seeed-Solution/rebot-companion.
