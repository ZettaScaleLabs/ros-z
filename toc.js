// Populate the sidebar
//
// This is a script, and not included directly in the page, to control the total size of the book.
// The TOC contains an entry for each page, so if each page includes a copy of the TOC,
// the total size of the page becomes O(n**2).
class MDBookSidebarScrollbox extends HTMLElement {
    constructor() {
        super();
    }
    connectedCallback() {
        this.innerHTML = '<ol class="chapter"><li class="chapter-item expanded affix "><a href="introduction.html">Introduction</a></li><li class="chapter-item expanded affix "><li class="part-title">Getting Started</li><li class="chapter-item expanded "><a href="chapters/quick_start.html"><strong aria-hidden="true">1.</strong> Quick Start</a></li><li class="chapter-item expanded affix "><li class="part-title">Core Concepts</li><li class="chapter-item expanded "><a href="chapters/pubsub.html"><strong aria-hidden="true">2.</strong> Pub/Sub</a></li><li class="chapter-item expanded "><a href="chapters/services.html"><strong aria-hidden="true">3.</strong> Services</a></li><li class="chapter-item expanded "><a href="chapters/actions.html"><strong aria-hidden="true">4.</strong> Actions</a></li><li class="chapter-item expanded affix "><li class="part-title">User Guide</li><li class="chapter-item expanded "><a href="chapters/building.html"><strong aria-hidden="true">5.</strong> Building</a></li><li class="chapter-item expanded "><a href="chapters/distro_compatibility.html"><strong aria-hidden="true">6.</strong> ROS 2 Distribution Compatibility</a></li><li class="chapter-item expanded "><a href="chapters/examples.html"><strong aria-hidden="true">7.</strong> Running Examples</a></li><li class="chapter-item expanded "><a href="chapters/networking.html"><strong aria-hidden="true">8.</strong> Networking</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="chapters/config_options.html"><strong aria-hidden="true">8.1.</strong> Configuration Options</a></li><li class="chapter-item "><a href="chapters/config_advanced.html"><strong aria-hidden="true">8.2.</strong> Advanced Configuration</a></li></ol></li><li class="chapter-item expanded "><a href="chapters/message_generation.html"><strong aria-hidden="true">9.</strong> Message Generation</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="chapters/custom_messages.html"><strong aria-hidden="true">9.1.</strong> Custom Messages</a></li><li class="chapter-item "><a href="chapters/protobuf.html"><strong aria-hidden="true">9.2.</strong> Protobuf Serialization</a></li></ol></li><li class="chapter-item expanded "><li class="part-title">Tools</li><li class="chapter-item expanded "><a href="chapters/console.html"><strong aria-hidden="true">10.</strong> ros-z-console</a></li><li class="chapter-item expanded affix "><li class="part-title">Experimental</li><li class="chapter-item expanded "><a href="chapters/shm.html"><strong aria-hidden="true">11.</strong> Shared Memory (SHM)</a></li><li class="chapter-item expanded "><a href="chapters/rmw_zenoh_rs.html"><strong aria-hidden="true">12.</strong> rmw_zenoh_rs</a></li><li class="chapter-item expanded "><a href="chapters/python.html"><strong aria-hidden="true">13.</strong> Python Bindings</a><a class="toggle"><div>❱</div></a></li><li><ol class="section"><li class="chapter-item "><a href="chapters/python_codegen.html"><strong aria-hidden="true">13.1.</strong> Code Generation Internals</a></li></ol></li><li class="chapter-item expanded "><li class="part-title">Appendix</li><li class="chapter-item expanded "><a href="chapters/nix.html"><strong aria-hidden="true">14.</strong> Reproducible Development with Nix</a></li><li class="chapter-item expanded "><a href="chapters/feature_flags.html"><strong aria-hidden="true">15.</strong> Feature Flags</a></li><li class="chapter-item expanded "><a href="chapters/troubleshooting.html"><strong aria-hidden="true">16.</strong> Troubleshooting</a></li></ol>';
        // Set the current, active page, and reveal it if it's hidden
        let current_page = document.location.href.toString().split("#")[0].split("?")[0];
        if (current_page.endsWith("/")) {
            current_page += "index.html";
        }
        var links = Array.prototype.slice.call(this.querySelectorAll("a"));
        var l = links.length;
        for (var i = 0; i < l; ++i) {
            var link = links[i];
            var href = link.getAttribute("href");
            if (href && !href.startsWith("#") && !/^(?:[a-z+]+:)?\/\//.test(href)) {
                link.href = path_to_root + href;
            }
            // The "index" page is supposed to alias the first chapter in the book.
            if (link.href === current_page || (i === 0 && path_to_root === "" && current_page.endsWith("/index.html"))) {
                link.classList.add("active");
                var parent = link.parentElement;
                if (parent && parent.classList.contains("chapter-item")) {
                    parent.classList.add("expanded");
                }
                while (parent) {
                    if (parent.tagName === "LI" && parent.previousElementSibling) {
                        if (parent.previousElementSibling.classList.contains("chapter-item")) {
                            parent.previousElementSibling.classList.add("expanded");
                        }
                    }
                    parent = parent.parentElement;
                }
            }
        }
        // Track and set sidebar scroll position
        this.addEventListener('click', function(e) {
            if (e.target.tagName === 'A') {
                sessionStorage.setItem('sidebar-scroll', this.scrollTop);
            }
        }, { passive: true });
        var sidebarScrollTop = sessionStorage.getItem('sidebar-scroll');
        sessionStorage.removeItem('sidebar-scroll');
        if (sidebarScrollTop) {
            // preserve sidebar scroll position when navigating via links within sidebar
            this.scrollTop = sidebarScrollTop;
        } else {
            // scroll sidebar to current active section when navigating via "next/previous chapter" buttons
            var activeSection = document.querySelector('#sidebar .active');
            if (activeSection) {
                activeSection.scrollIntoView({ block: 'center' });
            }
        }
        // Toggle buttons
        var sidebarAnchorToggles = document.querySelectorAll('#sidebar a.toggle');
        function toggleSection(ev) {
            ev.currentTarget.parentElement.classList.toggle('expanded');
        }
        Array.from(sidebarAnchorToggles).forEach(function (el) {
            el.addEventListener('click', toggleSection);
        });
    }
}
window.customElements.define("mdbook-sidebar-scrollbox", MDBookSidebarScrollbox);
