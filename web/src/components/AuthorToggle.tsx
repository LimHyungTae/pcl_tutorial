import { useState } from "react";
import { useT } from "../i18n";

export default function AuthorToggle() {
  const t = useT();
  const [open, setOpen] = useState(false);

  return (
    <div className="mt-8 flex flex-col items-center">
      <button
        onClick={() => setOpen((o) => !o)}
        aria-expanded={open}
        className={`code-font rounded-full border px-4 py-1.5 text-[11px] font-semibold uppercase tracking-[0.2em] transition ${
          open
            ? "border-[color:rgba(0,212,170,0.5)] bg-[color:rgba(0,212,170,0.1)] text-[var(--accent)]"
            : "border-[var(--border-strong)] bg-[var(--surface)] text-[var(--dim)] hover:border-[var(--accent)] hover:text-[var(--accent)]"
        }`}
      >
        {open ? `× ${t.author.toggle}` : t.author.toggle}
      </button>

      {open && (
        <div className="fade-up mt-5 w-full max-w-3xl rounded-xl border border-[var(--border)] bg-[var(--surface)] p-6 text-left">
          <div className="flex items-baseline justify-between gap-3">
            <h3 className="heading-mono text-xl font-bold text-[var(--text-strong)]">
              {t.author.title}
            </h3>
            <span className="code-font text-[11px] text-[var(--dim)]">
              {t.author.role}
            </span>
          </div>
          <p className="mt-2 text-[12px] italic leading-relaxed text-[var(--accent)]">
            {t.author.focus}
          </p>
          <p className="mt-3 text-[13.5px] leading-relaxed text-[var(--dim)]">
            {t.author.bio}
          </p>

          <div className="mt-5">
            <div className="code-font mb-2 text-[10px] font-semibold uppercase tracking-[0.2em] text-[var(--mut)]">
              {t.author.libsLabel}
            </div>
            <div className="flex flex-wrap gap-2">
              {t.author.libs.map((lib) => (
                <a
                  key={lib.name}
                  href={lib.url}
                  target="_blank"
                  rel="noreferrer"
                  className="code-font inline-flex items-center gap-1 rounded-md border border-[var(--border-strong)] bg-[var(--surface-2)] px-2.5 py-1 text-[11px] font-bold text-[var(--text)] transition hover:border-[var(--accent)] hover:text-[var(--accent)]"
                >
                  {lib.name}
                  <span className="text-[var(--mut)]">↗</span>
                </a>
              ))}
            </div>
          </div>

          <div className="mt-5 flex flex-wrap gap-2">
            <ContactLink label={t.author.blog} href={t.author.blogUrl} />
            <ContactLink label={t.author.github} href={t.author.githubUrl} />
            <ContactLink
              label={t.author.email}
              href={`mailto:${t.author.emailAddress}`}
            />
          </div>
        </div>
      )}
    </div>
  );
}

function ContactLink({ label, href }: { label: string; href: string }) {
  return (
    <a
      href={href}
      target="_blank"
      rel="noreferrer"
      className="code-font inline-flex items-center rounded-md border border-[var(--border-strong)] bg-[var(--surface-2)] px-2.5 py-1 text-[11px] text-[var(--dim)] transition hover:border-[var(--accent-2)] hover:text-[var(--accent-2)]"
    >
      {label}
      <span className="ml-1 text-[var(--mut)]">↗</span>
    </a>
  );
}
