# Security Policy

## Reporting Security Issues

If you discover a security vulnerability, please report it responsibly. **Do not** open a public issue.

### How to Report

Send an email to the repository maintainers with:

- Description of the vulnerability
- Steps to reproduce (if applicable)
- Potential impact
- Suggested fix (if you have one)

**Email:** security@ros2-builders.org (or replace with actual email)

### What to Expect

- We will acknowledge receipt within 48 hours
- We will provide a detailed response within 7 days
- We will work with you to coordinate a fix
- We will credit you in the security advisory (if desired)

### Supported Versions

| Version | Security Support |
|---------|------------------|
| Latest release | ✅ Supported |
| Previous release | ✅ Supported (if within 6 months) |
| Older releases | ❌ Unsupported |

## Security Features

### Automated Security Scanning

This project uses multiple security scanning tools:

| Tool | Purpose | Coverage |
|------|---------|----------|
| **CodeQL** | Static code analysis | Source code |
| **Trivy** | Vulnerability scanning | Files, containers |
| **Grype** | Container vulnerability scanning | Docker images |
| **Semgrep** | Custom SAST rules | Source code |
| **Syft** | SBOM generation | All builds |
| **Dependabot** | Dependency updates | Package files |

### Vulnerability Scanning Process

1. **On every push/PR**
   - CodeQL analysis runs
   - Trivy scans source files
   - Semgrep runs custom rules

2. **On Docker image builds**
   - Grype scans final images
   - High/critical CVEs fail the build
   - SBOM generated and uploaded

3. **Weekly scheduled scans**
   - Comprehensive vulnerability scan
   - Dependency updates checked
   - Security advisories reviewed

### SBOM Generation

Software Bill of Materials (SBOM) is generated for all releases:

- **Format**: SPDX-JSON and CycloneDX-JSON
- **Upload**: Attached to GitHub releases
- **Retention**: 90 days for artifacts
- **Access**: Publicly available for auditing

## Dependency Management

### Dependency Updates

Automated dependency updates are handled by Dependabot:

- **Weekly checks** for GitHub Actions
- **Weekly checks** for Python packages
- **Pull requests** automatically created
- **Auto-merge** enabled for minor/patch updates

### Reviewing Dependencies

When reviewing dependencies:

1. Check for known vulnerabilities
2. Verify maintainer reputation
3. Review change log for breaking changes
4. Test functionality after update

### Updating Dependencies

1. Dependabot creates PR with:
   - Change summary
   - Release notes
   - Vulnerability report

2. Review process:
   - Check test results
   - Review code changes
   - Test locally if needed
   - Approve or request changes

3. Merging:
   - Minor/patch: Auto-merged if tests pass
   - Major: Manual review required

## Secure Development Practices

### Code Guidelines

1. **No secrets in code**
   - Never commit passwords, tokens, or API keys
   - Use environment variables for sensitive data
   - Use GitHub Secrets for CI/CD

2. **Input validation**
   - Validate all user inputs
   - Sanitize data from external sources
   - Use parameterized queries

3. **Least privilege**
   - Use minimal permissions for workflows
   - Scope secrets to necessary access
   - Review permissions regularly

4. **Dependency hygiene**
   - Keep dependencies up-to-date
   - Remove unused dependencies
   - Use pinned versions where appropriate

### Docker Security

1. **Base images**
   - Use official images when possible
   - Prefer minimal/slim variants
   - Update base images regularly

2. **Image hardening**
   - Don't run as root if possible
   - Use read-only filesystems
   - Enable security options

3. **Vulnerability scanning**
   - Scan images before pushing
   - Fix high/critical CVEs
   - Track SBOMs for auditing

### CI/CD Security

1. **Workflow permissions**
   - Use minimal required permissions
   - Scope secrets appropriately
   - Use OIDC for cloud access

2. **Third-party actions**
   - Pin action versions
   - Review popular actions
   - Monitor for updates

3. **Artifact security**
   - Scan uploaded artifacts
   - Use checksums for integrity
   - Set appropriate retention

## Security Best Practices for Contributors

### Before Submitting Code

1. **Scan for secrets**
   ```bash
   # Use gitleaks to check for secrets
   gitleaks detect --source .
   ```

2. **Run security scanners**
   ```bash
   # Trivy scan
   trivy fs .

   # CodeQL (locally)
   codeql database create .
   codeql database analyze .
   ```

3. **Review dependencies**
   - Check for new package additions
   - Verify versions are appropriate
   - Update package files if needed

### During Code Review

1. Look for:
   - Hardcoded secrets
   - SQL injection vectors
   - XSS vulnerabilities
   - Unsafe deserialization
   - Weak cryptography

2. Verify:
   - Input validation
   - Output encoding
   - Error handling
   - Logging (no sensitive data)

## Incident Response

### Severity Levels

| Severity | Response Time | Description |
|----------|---------------|-------------|
| **Critical** | < 24 hours | Active exploitation, data loss |
| **High** | < 48 hours | Potential exploitation, high impact |
| **Medium** | < 1 week | Limited exploitability, medium impact |
| **Low** | < 2 weeks | Minor issues, low impact |

### Response Process

1. **Triage**
   - Confirm vulnerability
   - Assess severity
   - Determine impact

2. **Containment**
   - Mitigate immediate risk
   - Update affected systems
   - Notify stakeholders

3. **Remediation**
   - Develop fix
   - Test thoroughly
   - Deploy patch

4. **Post-Incident**
   - Document incident
   - Review response
   - Update processes

## Security Audits

### External Audits

- Frequency: Annually or as needed
- Scope: Full repository and infrastructure
- Disclosure: Public summary after fix

### Internal Reviews

- Frequency: Quarterly
- Scope: Recent changes and dependencies
- Disclosure: Team only

## Compliance

### Standards

This project aims to comply with:

- **OWASP Top 10** - Web application security
- **CISA SBOM** - Software supply chain security
- **SSDF** - Secure Software Development Framework

### Licenses

- **Code**: Apache-2.0
- **Dependencies**: Various per-package licenses
- **SBOM**: SPDX format for license tracking

## Additional Resources

- [GitHub Security Documentation](https://docs.github.com/en/security)
- [OWASP Cheat Sheets](https://cheatsheetseries.owasp.org/)
- [CISA Security Best Practices](https://www.cisa.gov/known-exploited-vulnerabilities-catalog)
- [NIST Cybersecurity Framework](https://www.nist.gov/cyberframework)

## Questions

For security-related questions:

- Send email to security@ros2-builders.org
- Use the `security` label in GitHub issues (for non-critical concerns)
- Check existing security discussions

Thank you for helping keep ROS2 Builders secure!
