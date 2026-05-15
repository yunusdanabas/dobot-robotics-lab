"""Windows-only MG400 helper text imported lazily by shared modules."""

from __future__ import annotations


def get_direct_connect_help_lines() -> list[str]:
    """Return Windows-specific MG400 direct-connect guidance."""
    return [
        "Windows PowerShell (from repo root):",
        "  .\\windows\\Set-MG400StaticIp.ps1   # dry run only",
        "  Get-NetAdapter",
        "  Run PowerShell as Administrator for the -Apply step",
        "  .\\windows\\Set-MG400StaticIp.ps1 -InterfaceAlias '<EthernetName>' -Apply",
        "  The PowerShell helper does not change the adapter unless you add -Apply.",
    ]
