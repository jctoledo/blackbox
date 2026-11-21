# Discovery research: Blackbox project website

Reviewed 2026-11-21. Scope: official Google Search and Bing guidance for the public `https://jctoledo.github.io/blackbox/` project site. These are recommendations, not evidence that pages have been indexed or that traffic will increase. No local builds or tests were run.

## Repository work

1. **Answer actual builder questions.** Prioritize useful build, wiring/mount alignment, GPS logging, lap timing, recording, calibration, and troubleshooting guides. Explain supported hardware, limitations, and evidence for capabilities. This page selection is a project-specific recommendation: Google recommends original, helpful, organized content and the terms readers use in prominent headings. Avoid creating near-duplicate pages for keyword variations. [Google SEO Starter Guide](https://developers.google.com/search/docs/fundamentals/seo-starter-guide), [Search Essentials](https://developers.google.com/search/docs/essentials)

2. **Give every guide a specific title, summary, and route.** Examples: “Build an ESP32-C3 GPS lap timer | Blackbox” and “GPS logging and recordings | Blackbox,” provided those pages actually cover those tasks. Keep important explanations in accessible HTML text; an interactive dashboard needs a textual explanation alongside it. Google uses titles/headings for title links and may use a unique meta description for the snippet; these tags influence presentation without fixing the exact wording Google displays. [Google SEO Starter Guide](https://developers.google.com/search/docs/fundamentals/seo-starter-guide), [Developer guidance](https://developers.google.com/search/docs/fundamentals/get-started-developers)

3. **Connect the site with ordinary links.** Link from the homepage to the main guides, from guides to prerequisites and related instructions, and from the repository README/About website field to the public site. Use descriptive link text such as “Mount and align the IMU.” Google recommends that pages be reachable from another findable page; links help discovery and user navigation. [Google Developer guidance](https://developers.google.com/search/docs/fundamentals/get-started-developers), [SEO Starter Guide](https://developers.google.com/search/docs/fundamentals/seo-starter-guide)

4. **Maintain the existing sitemap.** Publish preferred, absolute public URLs under `/blackbox/`, including newly generated guides, and keep canonical URLs consistent. Omit unpublished pages and duplicate variants. `lastmod` is optional; include it only when it reflects a real significant update. Google ignores `priority` and `changefreq`. A sitemap submission is a discovery hint, not an indexing guarantee. [Google sitemap guidance](https://developers.google.com/search/docs/crawling-indexing/sitemaps/build-sitemap)

5. **Keep crawler policy at the correct scope.** Do not add `docs/robots.txt` as a discovery fix: the deployed `/blackbox/robots.txt` cannot control host crawling. An effective file belongs at `https://jctoledo.github.io/robots.txt`, requiring coordination with the root site owner if policies need changing. Google allows crawling by default absent blocking rules. [Google robots.txt location and defaults](https://developers.google.com/crawling/docs/robots-txt/create-robots-txt)

## Video

The decorative circuit hero video is not a product demonstration or a dedicated watch page. Do not expect it to earn video results. If a real hardware/dashboard demonstration becomes available, an optional dedicated page can make that single video the main content, with its own title, description, stable thumbnail/media URLs, and supporting transcript. Use ordinary video/player HTML and accurate video metadata; do not load the player only after interaction if discovery matters. Google requires an indexed watch page and other conditions for video indexing; indexing the page alone is insufficient. [Google video best practices](https://developers.google.com/search/docs/appearance/video)

## AI search expectations

Google applies normal Search eligibility to AI Overviews/AI Mode: a supporting page must be indexed and eligible for a snippet. It calls for helpful textual content, internal links, and structured data matching visible content. No additional AI text files or special schema are required. Treat the existing `llms.txt` as an optional reading index, with no promised discovery, ranking, citation, or traffic benefit. Google reports these AI features within Search Console's overall Web performance. [Google AI features guidance](https://developers.google.com/search/docs/appearance/ai-features)

Bing likewise describes ordinary crawling, indexing, content clarity, and accuracy as foundations for Copilot and grounding eligibility. Keep technical explanations understandable independently and support media with text. Do not fabricate benchmarks or write claims aimed at triggering citations. [Bing Webmaster Guidelines](https://www.bing.com/webmasters/help/bing-webmaster-guidelines-30fba23a)

## Owner/account setup after deployment

These actions require the owner's accounts and verification; repository edits alone cannot complete them.

1. Add the exact URL-prefix property `https://jctoledo.github.io/blackbox/` to Google Search Console. A URL-prefix property can cover a path; a Domain property requires DNS verification and is a different scope. Use the verification method Search Console offers, such as an HTML meta tag in the homepage, and retain the real token. Do not invent a placeholder verification claim. [URL-prefix properties](https://support.google.com/webmasters/answer/10432366), [Property setup](https://support.google.com/webmasters/answer/34592), [Ownership verification](https://support.google.com/webmasters/answer/9008080)

2. Submit `https://jctoledo.github.io/blackbox/sitemap.xml` in that property's Sitemaps report. Inspect the homepage and representative deployed guides with URL Inspection: check live rendering/resources and the indexed canonical, then optionally request indexing. A successful live test does not prove indexing or search appearance. [Sitemap submission](https://developers.google.com/search/docs/crawling-indexing/sitemaps/build-sitemap), [URL Inspection](https://support.google.com/webmasters/answer/9012289)

3. Add/verify the site in Bing Webmaster Tools and submit the same sitemap through the owner's account. Review indexing/crawl issues before expanding the content inventory. [Bing getting started](https://www.bing.com/webmasters/help/getting-started-checklist-66a806de), [Bing sitemaps](https://www.bing.com/webmasters/help/sitemaps-3b5cf6ed)

4. Measure real outcomes: indexed guide URLs, impressions, clicks, and relevant queries, with the date of deployment recorded for comparison. Neither engine promises inclusion, ranking, or referrals. [Google AI features guidance](https://developers.google.com/search/docs/appearance/ai-features)

## Recommended order

Keep existing technical guides and metadata coherent; make the build and operation paths easy to follow; update the sitemap with deployed routes. Then complete owner verification/submission and use the reports to identify real discovery problems. Add a demo watch page only when there is an authentic explanatory video to publish.
