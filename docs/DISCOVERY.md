# Search and AI discovery

The project publishes server-rendered HTML content, descriptive titles and summaries, canonical URLs, Open Graph metadata, SoftwareSourceCode structured data, static technical guides, and a sitemap at https://jctoledo.github.io/blackbox/sitemap.xml. The optional llms.txt is a reading index; it is not a Google requirement or a ranking promise.

Google's [AI features guidance](https://developers.google.com/search/docs/appearance/ai-features) says ordinary Search eligibility and helpful, accessible content apply. There is no special AI schema that guarantees inclusion. See also [sitemap guidance](https://developers.google.com/search/docs/crawling-indexing/sitemaps/build-sitemap).

After deployment, the owner can verify the `https://jctoledo.github.io/blackbox/` URL-prefix property in Search Console, submit `sitemap.xml`, and inspect the homepage and guide URLs. Account verification and indexing decisions are outside repository code. No submission or ranking improvement is claimed by these changes.

GitHub project Pages is hosted below `/blackbox/`. An effective robots.txt must live at the origin root (`https://jctoledo.github.io/robots.txt`), which is outside this repository's deployment path. A `/blackbox/robots.txt` would not control crawlers, so this project does not pretend that it does. Coordinate root policies with the owner's root Pages site if needed; ordinary public access does not require an Allow file.

Keep claims consistent across README, About, structured data, and the website. Regenerate static guides with `python3 scripts/build_docs.py` (Markdown 3.7); Pages workflows do this automatically. Do not add unsupported ratings, invented benchmarks, keyword stuffing, or hidden bot-only claims.

Use [COMPARISON.md](COMPARISON.md) as the public capability/evidence reference. Lead with what users can do; distinguish implementation, configuration targets, and measured results. Keep vendor comparisons sourced and dated. Proposed acceleration tests, slope correction, replay, and coaching must not appear as available features.
