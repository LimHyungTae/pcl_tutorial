import { useT } from "../i18n";

/** Brief paragraph describing what this demo does — main area, under the chapter header. */
export default function DemoAbout({ slug }: { slug: string }) {
  const t = useT();
  const c = t.chapters[slug as keyof typeof t.chapters] as
    | { about?: string }
    | undefined;
  if (!c?.about) return null;
  return (
    <p className="mt-5 max-w-4xl text-[14px] leading-relaxed text-[var(--dim)]">
      {c.about}
    </p>
  );
}
